#include "t265_connect.h"

T265_Connect::T265_Connect(std::string orientation)
{
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    CamOrient = orientation;

    time_to_exit = false;
    read_status = false;
}


T265_Connect::T265_Connect()
{
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    CamOrient = "forward";

    time_to_exit = false;
    read_status = false;
}


bool T265_Connect::getData(vehicleState *X)
{
    mu.lock();
    X->timestamp = pose_data.timestamp;
    X->pos.X     = pose_data.pos.X;
    X->pos.Y     = pose_data.pos.Y;
    X->pos.Z     = pose_data.pos.Z;
    X->quat.Q0   = pose_data.quat.Q0;
    X->quat.Q1   = pose_data.quat.Q1;
    X->quat.Q2   = pose_data.quat.Q2;
    X->quat.Q3   = pose_data.quat.Q3;
    mu.unlock();
    return true;
}


void T265_Connect::setData(vehicleState *X)
{
    mu.lock();
    pose_data.timestamp = X->timestamp;
    pose_data.pos.X     = X->pos.X;
    pose_data.pos.Y     = X->pos.Y;
    pose_data.pos.Z     = X->pos.Z;
    pose_data.quat.Q0   = X->quat.Q0;
    pose_data.quat.Q1   = X->quat.Q1;
    pose_data.quat.Q2   = X->quat.Q2;
    pose_data.quat.Q3   = X->quat.Q3;
    mu.unlock();
}


void T265_Connect::read_RS()
{
    if (read_status)
    {
        fprintf(stderr,"read thread already running");
    }else
    {
        pipe.start(cfg);

        while(read_status){

            vehicleState pose;
            readPose(&pose);
            setData(&pose);
        }
    }
}


void T265_Connect::startThread()
{
    if(!read_status){

        read_status = true;
        readThread = std::thread(&T265_Connect::read_RS, this);

    }
}


void T265_Connect::stopThread()
{
    time_to_exit = true;

    if(read_status){

        std::cout << "\nCLOSING T265 READ THREAD";

        read_status = false;
        if(readThread.joinable()){
            readThread.join();
        }
    }
}

void T265_Connect::readPose(vehicleState *pose)
{
    if(!time_to_exit){
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        /*if (rs2::pose_frame fp = frames.as<rs2::pose_frame>()) {*/
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        double camX = pose_data.translation.x;
        double camY = pose_data.translation.y;
        double camZ = pose_data.translation.z;

        rs2_time_t frame_time = f.get_timestamp();
        pose->timestamp = static_cast<uint64_t>(frame_time+0.5);

        double q[4];
        q[0] = pose_data.rotation.w;
        q[1] = pose_data.rotation.x;
        q[2] = pose_data.rotation.y;
        q[3] = pose_data.rotation.z;

        double X[3];
        X[0] = camX;
        X[1] = camY;
        X[2] = camZ;
        double corrq[4], corrX[3];
        poseTrans(CamOrient,q,X,corrq,corrX);
        pose->pos.X      = corrX[0];
        pose->pos.Y      = corrX[1];
        pose->pos.Z      = corrX[2];
        pose->quat.Q0    = corrq[0];
        pose->quat.Q1    = corrq[1];
        pose->quat.Q2    = corrq[2];
        pose->quat.Q3    = corrq[3];

    }
}


// -------------------------------------------------------------------
//   Quit Handler
// -------------------------------------------------------------------
void T265_Connect::handle_quit()
{
    try{
        stopThread();
    }
    catch (int error) {
        fprintf(stderr,"\nWarning, could not stop RS interface\n");
    }
}


inline rs2_quaternion quaternion_exp(rs2_vector v)
{
    float x = v.x/2, y = v.y/2, z = v.z/2, th2, th = sqrtf(th2 = x*x + y*y + z*z);
    float c = cosf(th), s = th2 < sqrtf(120*FLT_EPSILON) ? 1-th2/6 : sinf(th)/th;
    rs2_quaternion Q = { s*x, s*y, s*z, c };
    return Q;
}

inline rs2_quaternion quaternion_multiply(rs2_quaternion a, rs2_quaternion b)
{
    rs2_quaternion Q = {
        a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z,
        a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z,
        a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
    return Q;
}

rs2_pose predict_pose(rs2_pose & pose, float dt_s)
{
    rs2_pose P = pose;
    P.translation.x = dt_s * (dt_s/2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
    P.translation.y = dt_s * (dt_s/2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
    P.translation.z = dt_s * (dt_s/2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;
    rs2_vector W = {
            dt_s * (dt_s/2 * pose.angular_acceleration.x + pose.angular_velocity.x),
            dt_s * (dt_s/2 * pose.angular_acceleration.y + pose.angular_velocity.y),
            dt_s * (dt_s/2 * pose.angular_acceleration.z + pose.angular_velocity.z),
    };
    P.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
    return P;
}

