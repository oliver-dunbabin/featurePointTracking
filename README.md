# featurePointTracking
Program which performs feature tracking using a monocular camera, using Harris corner detection within an EKF/UKF framework. The filter estimates landmark/obstacle locations in inertial space (formulated using inverse-depth parameterisation) by corresponding feature-point measurements obtained in each camera frame with a database/map of estimated points. This mapper is designed to be used standalone, or as one half of the SLAM solution.

The project is organised as follows:
- **binaryConverter/**: Takes binary files recorded by hardware operation of filter and converts them to human-readable .txt file.
- **HarrisCorner/**: The source code for the Harris corner algorithm, ready to be created into a module for the Jevois camera.
- **onboard/**: The onboard code which runs the filter. This code can be run in simulation mode (where binary files are used as the navigation/camera inputs into the filter, and results are plotted) or in hardware mode (where Intel RealSense T265 odometry unit gives vehicle pose, and Jevois camera extracts feature-points from the camera frames - using Vicon motion tracking system for validation is optional).
- **testdata/**: Some binary files of past tests using this mapping algorithm.

## Contributing

To contribute to this codebase, common version control practices should be observed.

### Cloning Repo
Get a local version of this repository.
```
git clone https://github.com/OllieD3711/featurePointTracking.git
```

### Making edits to the code
Everytime an edit is to be made to the code, it should first be made to a new branch on the local machine (off the origin/master branch). "To begin working on anything new in a project, or to change existing things, you create a branch off the stable master branch".

Prior to creating a new branch, you want to ensure your local master branch is up to date with the latest changes. CD into your local repository (the directory where you cloned this repo). Now pull the latest stable changes made to the master branch.
```
git pull
```

Once updated, you want to create a branch with which to make any local changes to the master code, allowing you to make edits and test, without changing the master ("stable") branch - give the branch a descriptive name.
```
git checkout -b name_of_new_branch
```

Check which is your current working branch to ensure you are making changes to your new branch only.
```
git branch
```

Now you are free to edit code as you wish in your local branch.


### Committing changes to your local branch
Stage and commit your changes to your current working branch.
```
git add your_edited_file.py
```

If you have forgotten which files you have modified (and need to be staged), you can see all files which have unstaged edits (these files will be coloured red).
```
git status
```

(If you have edited many files, you can add them all (new, modified, deleted))
```
git add -A
```
(Or)
```
git add .
```
(Only stage modified and deleted files)
```
git add -u
```
(Only stage new and modified files)
```
git add --ignore-removal .
```

Now commit these staged changes to your local branch.
```
git commit
```
Remember to write a informative description of the changes you have made, when prompted.
 
### Pushing changes to the master branch
If you are happy with your changes (and they are thoroughly tested and BUG FREE!!), you are ready to push these changes to the master branch.

Push your new local branch to the remote repository
```
git push origin name_of_new_branch
```

During the time it took to make these changes to your code, the master branch may have been edited by another contributer. Navigate to the remote repository in your browser and click on the tab "branches". This will show all the active branches which have been pushed to the remote repo (and the "Default branch" -> the master branch).

Create a new pull request by clicking "New pull request" and then "Create pull request". It is here that changes to your_edited_file.py can be seen (with deletions highlighted red and additions highlighted green).

If there are no conflicts between your branch and the remote master, you are able to merge by clicking "Merge pull request". 

Alternatively, this process could have been completed from command line.
```
git fetch origin
git checkout -b name_of_new_branch origin/name_of_new_branch
git merge master
```
and then
```
git checkout master
git merge --no-ff name_of_new_branch
git push origin master
``` 
(--no-ff indicates that we want to retain all of the commit messages prior to this merge).

If there are merge conflicts, the relevant lines of your_edited_file.py will be displayed, and left to you to reconcile the conflicts. Once reconciled, the process proceeds as described above.

### Deleting your branch
Since we have successfully merged your new branch with master, we no longer need it, and will delete.
```
git branch -d name_of_new_branch
```

Alternatively, the branch can be deleted from github in your browser (one method is to click on the "branches" tab and clicking the trash can next to the branch in question".
