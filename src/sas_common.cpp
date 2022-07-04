#include <sas_common/sas_common.h>
#include <mutex>

namespace sas
{
namespace common
{

/**
 * @brief get_static_node_handle a version of roscpp_initializer without using boost.
 * @return a reference to the global NodeHandle. Should only be used for the Python wrappers of the sas Classes.
 * Inspired on: https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning_interface/py_bindings_tools/src/roscpp_initializer.cpp
 */
ros::NodeHandle& get_static_node_handle()
{
    std::lock_guard<std::mutex> lock(node_handle_mutex);
    if(!ros::isInitialized())
    {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Initializing roscpp with a dummy name...");
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc,argv,"sas_common_nodehandle_dummy", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    }
    if(not node_handle)
    {
        node_handle = {new ros::NodeHandle(),
                       [](ros::NodeHandle* nh)
                       {
                           if (ros::isInitialized() && !ros::isShuttingDown())
                           {
                               ros::shutdown();
                           }
                           if(nh != nullptr)
                           {
                               delete nh;
                           }
                       }
                      };
       async_spinner.reset(new ros::AsyncSpinner(1));
       async_spinner->start();
    }
    return (*node_handle.get());
}
}
}
