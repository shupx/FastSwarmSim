# Instructions for Releasing a New Version to ROS distribution

Tutorial: https://docs.ros.org/en/lyrical/Developer-Tools/Build/Releasing/Releasing-a-Package.html

## First time release

See https://docs.ros.org/en/lyrical/Developer-Tools/Build/Releasing/Index-Your-Packages.html

1. [Create a PR to rosdistro repository](https://docs.ros.org/en/lyrical/Developer-Tools/Build/Releasing/Index-Your-Packages.html#make-changes-to-your-fork) to add the package `source` and `status` to the `distribution.yaml` file of the target ROS distribution, but without the `release` field. **Waiting for the rosdistro reviewer to merge the PR**. see [Index fastswarmsim PR](https://github.com/ros/rosdistro/pull/53487). The `ROS Index` will include the package once the PR is merged (see fastswarmsim in [ros index](https://index.ros.org/r/fastswarmsim/)). The package is not yet released to the target ROS distribution, and you cannot install it from the ROS build farm yet.

2. Once the rosdistro PR is merged with your new package, then [start a New Release Team Issue to the ros2-gbp/ros2-gbp-github-org repository](https://docs.ros.org/en/lyrical/Developer-Tools/Build/Releasing/Release-Team-Repository.html#start-a-new-release-team). I request to add a new team `fastswarmsim_team` with a member `shupx` (my github account) through the [#1110 issue](https://github.com/ros2-gbp/ros2-gbp-github-org/issues/1110). **Wait for the ros2-gbp reviewer to invite you to the ros2-gdp organization**.

3. Create a new issue to the ros2-gbp/ros2-gbp-github-org repository to [request a new release of the package](https://docs.ros.org/en/lyrical/Developer-Tools/Build/Releasing/Release-Team-Repository.html#create-a-new-release-repository), then **waiting for the ros2-gbp reviewer to create a new release repository under the ros2-gbp organization, and grant you the permission to push to the repository**. 
Since I have also requested a new release of the package `fastswarmsim` through the [#1111 issue](https://github.com/ros2-gbp/ros2-gbp-github-org/issues/1110), this step is skipped, and the reviewer has created a new release repository [fastswarmsim-release](https://github.com/ros2-gbp/fastswarmsim-release) for me under the ros2-gbp organization and granted me the permission to push to the repository.

4. Under the source repository (FastSwarmSim), finish steps from `Install dependencies` to `Bloom Release` in the [First-Time-Release Tutorial](https://docs.ros.org/en/lyrical/Developer-Tools/Build/Releasing/First-Time-Release.html). Then, the source repository `FastSwarmSim` is tagged with the new version, the release repository `fastswarmsim-release` is automatically created with the new version branch, and a new PR is automatically created to the rosdistro repository to add the `release` field to the `distribution.yaml` file of the target ROS distribution. **Waiting for the rosdistro reviewer to merge the PR**. 

5. Once the rosdistro PR is merged with your new release, then the new version of the package is officially released to the target ROS distribution, and your packages will become available in the ROS build farm in 24-48 hours, and you can [test the pre-release binaries by installing from the ros2-testing-apt-source](https://docs.ros.org/en/lyrical/Developer-Tools/Debugging/Testing/Testing.html) if the package is built successfully on the ROS build farm. 


6. Approximately every two to four weeks, [the distribution’s release manager manually synchronizes the contents of ros-testing into the main ROS repository](https://discourse.openrobotics.org/c/ros/release/16). This is when your packages actually become available from ros2 apt sources. 


### Trouble shooting


bloom-release needs you to respond some values

```bash
bloom-release --new-track --rosdistro humble --track humble fastswarmsim
```

Respond with the following values, with all other values left as default by pressing `Enter`:

```bash
Release Repository url:   https://github.com/ros2-gbp/fastswarmsim-release.git
Repository Name:          fastswarmsim
Upstream Repository URI:  https://github.com/shupx/FastSwarmSim.git
Upstream Devel Branch:    main
```

The Bloom release process may fail to generate the PR to the rosdistro repository, since I use a ustc mirror of the rosdep. 

If all other steps are completed successfully, you can rerun with

```bash
# only generate the PR to the rosdistro repository, without redoing the bloom release process

ROSDISTRO_INDEX_URL=https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml bloom-release --rosdistro humble --track humble fastswarmsim --pull-request-only
```

## Subsequent release

Refer to https://docs.ros.org/en/lyrical/Developer-Tools/Build/Releasing/Subsequent-Releases.html

```bash
sudo rosdep init
rosdep update

cd FastSwarmSim/
git pull

# auto generate changelog based on git commit history since the last release
catkin_generate_changelog
git add .
git commit -m "update changelog"
git push

# auto update the version in package.xml, and tag the new version
catkin_prepare_release

# bloom release (push to fastswarmsim-release repository, and generate a PR to the rosdistro repository automatically)
ROSDISTRO_INDEX_URL=https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml bloom-release --rosdistro humble fastswarmsim
```

Then **waiting for the rosdistro reviewer to merge the PR** to the rosdistro repository. Once the PR is merged, then the new version of the package is officially released to the target ROS distribution, and your packages will become available in the ROS build farm in 24-48 hours, and you can [test the pre-release binaries by installing from the ros2-testing-apt-source](https://docs.ros.org/en/lyrical/Developer-Tools/Debugging/Testing/Testing.html) if the package is built successfully on the ROS build farm.

Approximately every two to four weeks, [the distribution’s release manager manually synchronizes the contents of ros-testing into the main ROS repository](https://discourse.openrobotics.org/c/ros/release/16). This is when your packages actually become updated from ros2 apt sources. 