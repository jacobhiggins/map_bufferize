#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class NodeLogic{
    public:
        ros::NodeHandle nh;
        ros::Subscriber map_in_sub;
        ros::Publisher map_out_pub;
        double buffer_size;
        NodeLogic(const ros::NodeHandle& nh_, const double& buffer_size_) : nh(nh_), buffer_size(buffer_size_){
            map_in_sub = nh.subscribe("/map", 1, &NodeLogic::map_in_callback, this);
            map_out_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map_bufferized", 1, true);
        }
        void map_in_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
            nav_msgs::OccupancyGrid map_out = *msg;
            unsigned int zero = 0;

            int buffer_size_idx = int(buffer_size / msg->info.resolution);

            for(int i = 0; i < msg->info.width; i+=1){
                for(int j = 0; j < msg->info.height; j+=1){
                    // (i,j) is the buffer test point
                    if (msg->data[j * msg->info.width + i] == 100 || msg->data[j * msg->info.width + i] == -1){continue;}

                    double dist = 1e6;
                    double stop = false;
                     
                    bool flipl = false;
                    for (int l = 0; l < buffer_size_idx; l++){
                        int j_new = (flipl) ? j+l : j-l;
                        flipl = !flipl;
                        bool flipk = false;
                        j_new = std::max(0, std::min(j_new, int(msg->info.height) - 1));
                        for (int k = 0; k < buffer_size_idx; k++){
                            int i_new = (flipk) ? i+k : i-k; flipk = !flipk;
                            i_new = std::max(0, std::min(i_new, int(msg->info.width) - 1));
                            
                            int occ_val = msg->data[j_new * msg->info.width + i_new];
                            if (occ_val == 100 || occ_val == -1){
                                dist = std::min(dist,sqrt(double(pow(i_new - i, 2)) + double(pow(j_new - j, 2))));
                                // dist = sqrt(2)*double(buffer_size_idx); //sqrt(pow(double(i_new - i), 2) + pow(double(j_new - j), 2));
                                // dist = std::abs(i_new - i);
                                // stop = true;
                            }
                            if (stop){break;}
                            if (!ros::ok()){return;}
                        }
                        if (stop){break;}
                    }

                    if (dist > 1000){
                        map_out.data[j * msg->info.width + i] = 0;
                    } else {
                        double frac = dist / (std::sqrt(2.0)*double(buffer_size_idx)); frac = std::min(1.0, frac);
                        map_out.data[j * msg->info.width + i] = int(((1-frac) * 100));
                    }
                    
                }
            }
            std::cout << "Bufferized map" << std::endl;
            ros::Duration(0.1).sleep();
            map_out_pub.publish(map_out);
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "map_bufferize_node");
    ros::NodeHandle nh;

    double buffer_size;
    if(!nh.getParam("map_bufferize_node/buffer_size",buffer_size)){ROS_ERROR("Failed to get param 'buffer_size'"); return -1;}

    NodeLogic nl(nh, buffer_size);

    ros::spin();
    return 0;
}