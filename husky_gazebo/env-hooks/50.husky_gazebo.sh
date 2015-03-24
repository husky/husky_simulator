if [ -z "$HUSKY_GAZEBO_DESCRIPTION" ]; then
  export HUSKY_GAZEBO_DESCRIPTION=$(catkin_find --first-only husky_gazebo urdf/description.gazebo.xacro 2>/dev/null)
fi
