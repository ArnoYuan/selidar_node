#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "SelidarDriver.h"
#include "SelidarProtocol.h"
#include "SelidarTypes.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace NS_NaviCommon;
using namespace NS_Selidar;

SelidarDriver * drv = NULL;

void publish_scan(ros::Publisher *pub,
                  SelidarMeasurementNode *nodes,
                  size_t node_count, ros::Time start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  std::string frame_id)
{
    static int scan_count = 0;
    sensor_msgs::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_count++;

    bool reversed = (angle_max > angle_min);
    if ( reversed ) {
      scan_msg.angle_min =  M_PI - angle_max;
      scan_msg.angle_max =  M_PI - angle_min;
    } else {
      scan_msg.angle_min =  M_PI - angle_min;
      scan_msg.angle_max =  M_PI - angle_max;
    }
    scan_msg.angle_increment =
        (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count-1);
    scan_msg.range_min = 0.15;
    scan_msg.range_max = 8.0;

    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);
    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
    if (!reverse_data) {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float) nodes[i].distance_scale_1000 / 1000.0f;
            if (read_value == 0.0)
                scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[i] = read_value;
        }
    } else {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float) nodes[i].distance_scale_1000 / 1000.0f;
            if (read_value == 0.0)
                scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[node_count-1-i] = read_value;
        }
    }
   /* for(int i = 0; i < node_count; i++)
    {
      printf("%f\n", nodes[i].angle_scale_100/ 100.0f);
    }  */
    pub->publish(scan_msg);
}
  bool
  checkSelidarHealth (SelidarDriver * drv)
  {
    int op_result;
    SelidarHealth healthinfo;
    
    op_result = drv->getHealth (healthinfo);
    
    if (IS_OK(op_result))
    {
      ROS_DEBUG ("Selidar health status : %d, errcode: %d",
                                    healthinfo.status, healthinfo.err_code);
      
      if (healthinfo.status != StatusFine)
      {
        ROS_DEBUG ("Selidar's status is not fine! ");
        return false;
      }
      else
      {
        ROS_DEBUG ("Selidar's status is not fine! ");
        return true;
      }
      
    }
    else
    {
      return false;
    }
  }
  
  bool
  checkSelidarInfo (SelidarDriver * drv)
  {
    int op_result;
    SelidarInfo device_info;
    
    op_result = drv->getDeviceInfo (device_info);
    
    if (IS_OK(op_result))
    {
      ROS_DEBUG ("Selidar device info :");
      ROS_DEBUG ("\t model : %d ", device_info.model);
      ROS_DEBUG ("\t hw ver : %d ", device_info.hw_id);
      ROS_DEBUG ("\t fw ver : %d.%d ", device_info.fw_major,
                                    device_info.fw_minor);
      return true;
    }
    else
    {
      return false;
    }
    
  }

bool stop_motor(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
  if(!drv)
       return false;

  ROS_DEBUG("Stop motor");
  drv->stop();
  return true;
}

bool start_motor(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
  if(!drv)
       return false;
  ROS_DEBUG("Start motor");
  drv->startScan();;
  return true;
}

void scanLoop (ros::Publisher scan_pub, bool inverted, std::string frame_id)
  {
    
    int op_result;
    ros::Time start_scan_time;
    ros::Time end_scan_time;
 
    double scan_duration;

    const int buffer_size = 360 * 10;
    SelidarMeasurementNode nodes_buffer[buffer_size];
    SelidarMeasurementNode nodes_temp[buffer_size];
    SelidarMeasurementNode nodes_pub[buffer_size];
    size_t buffered_nodes = 0;
    memset (nodes_buffer, 0, sizeof(nodes_buffer));
    memset (nodes_temp, 0, sizeof(nodes_temp));
    memset (nodes_pub, 0, sizeof(nodes_pub));

    while (ros::ok())
    {
      SelidarMeasurementNode nodes[buffer_size];
      size_t count = _countof(nodes);
      start_scan_time = ros::Time::now ();
      op_result = drv->grabScanData (nodes, count);
      end_scan_time = ros::Time::now();
      scan_duration = (end_scan_time - start_scan_time).toSec () * 1e-3;

      if (op_result == Success)
      {
        if ((buffered_nodes + count) > buffer_size)
        {
          printf("Lidar buffer is full!");
          buffered_nodes = 0;
          continue;
        }

        for (int i = 0; i < count; i++)
        {
          nodes_buffer[buffered_nodes] = nodes[i];
          if (nodes_buffer[buffered_nodes].angle_scale_100 >= 36000)
            nodes_buffer[buffered_nodes].angle_scale_100 -= 36000;
          buffered_nodes++;
        }

        int pub_nodes_count = 0;

        for (int i = 0; i < buffered_nodes; i++)
        {
          if (i > 0 && nodes_buffer[i].angle_scale_100 < nodes_buffer[i - 1].angle_scale_100)
          {
            pub_nodes_count = i;
            //copy publish nodes
            for (int j = 0; j < pub_nodes_count; j++)
            {
              nodes_pub[j] = nodes_buffer[j];
            }

            //re-serialize,save temp
            for (int j = 0; j < buffered_nodes; j++)
            {
              nodes_temp[j] = nodes_buffer[pub_nodes_count + j];
            }

            //save to buffer
            buffered_nodes -= pub_nodes_count;
            for (int j = 0; j < buffered_nodes; j++)
            {
              nodes_buffer[j] = nodes_temp[j];
            }

            break;
          }
        }

       /* if (pub_nodes_count > 0)
        {

          float angle_min = (nodes_pub[0].angle_scale_100 % 36000) / 100.0f;
          float angle_max = (nodes_pub[pub_nodes_count - 1].angle_scale_100 % 36000) / 100.0f;


          angle_min = DEG2RAD(angle_min);
          angle_max = DEG2RAD(angle_max);

          publish_scan(&scan_pub,
                  angle_compensate_nodes,
                  pub_nodes_count, start_scan_time,
                  scan_duration, inverted,
                  angle_min, angle_max,
                  frame_id);
        }

*/
        if (pub_nodes_count > 0)
        {
          float angle_min = DEG2RAD(0.0f);
          float angle_max = DEG2RAD(359.0f);

          const int angle_compensate_nodes_count = 360;
          const int angle_compensate_multiple = 1;
          int angle_compensate_offset = 0;
          SelidarMeasurementNode angle_compensate_nodes[angle_compensate_nodes_count];
          memset (angle_compensate_nodes, 0, angle_compensate_nodes_count * sizeof(SelidarMeasurementNode));

          int i, j;
         /* for (i = 0; i < pub_nodes_count; i++)
          {//with   angle_compensate
            if (nodes_pub[i].distance_scale_1000 != 0)
            {
              float angle = (float) (nodes_pub[i].angle_scale_100) / 100.0f;
              int angle_value = (int) (angle * angle_compensate_multiple);
              if ((angle_value - angle_compensate_offset) < 0)
                angle_compensate_offset = angle_value;

              for (j = 0; j < angle_compensate_multiple; j++)
              {
                angle_compensate_nodes[angle_value - angle_compensate_offset + j] = nodes_pub[i];
              }
            }
          }
          publish_scan(&scan_pub,
                  angle_compensate_nodes,
                  pub_nodes_count, start_scan_time,
                  scan_duration, inverted,
                  angle_min, angle_max,
                  frame_id);
        }*/
	for (i = 0; i < pub_nodes_count; i++)
          {//without   angle_compensate
              float angle = (float) (nodes_pub[i].angle_scale_100) / 100.0f;
              int angle_value = (int) (angle * angle_compensate_multiple);
              if ((angle_value - angle_compensate_offset) < 0)
              	angle_compensate_offset = angle_value;          
          }
		publish_scan(&scan_pub,
		          nodes_pub,
		          pub_nodes_count, start_scan_time,
		          scan_duration, inverted,
		          angle_min, angle_max,
		          frame_id);
        }
        /*
        float angle_min = DEG2RAD(0.0f);
        float angle_max = DEG2RAD(359.0f);
        
        const int angle_compensate_nodes_count = 360;
        const int angle_compensate_multiple = 1;
        int angle_compensate_offset = 0;
        SelidarMeasurementNode angle_compensate_nodes[angle_compensate_nodes_count];
        memset (angle_compensate_nodes, 0, angle_compensate_nodes_count * sizeof(SelidarMeasurementNode));

        int i, j;
        for (i = 0; i < count; i++)
        {
          if (nodes[i].distance_scale_1000 != 0)
          {
            float angle = (float) (nodes[i].angle_scale_100) / 100.0f;
            int angle_value = (int) (angle * angle_compensate_multiple);
            if ((angle_value - angle_compensate_offset) < 0)
              angle_compensate_offset = angle_value;

            for (j = 0; j < angle_compensate_multiple; j++)
            {
              angle_compensate_nodes[angle_value - angle_compensate_offset + j] = nodes[i];
            }
          }
        }
        publish_scan(&scan_pub,
                  angle_compensate_nodes,
                  pub_nodes_count, start_scan_time,
                  scan_duration, inverted,
                  angle_min, angle_max,
                  frame_id);
        */


      }
    }

  }
  
int main(int argc, char * argv[]) 
{

    ros::init(argc, argv, "selidar_node");

    std::string serial_port;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200); 
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("inverted", inverted, false);
    // create the driver instance
    drv = new SelidarDriver();
    
    if (!drv) {
        fprintf(stderr, "Create Driver fail, exit\n");
        return -2;
    }
    if (IS_FAIL(drv->connect(serial_port.c_str(), (uint32_t)serial_baudrate, 0))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , serial_port.c_str());
        drv->disconnect ();
        return -1;
    }

    NS_NaviCommon::delay (100);

   // ros::ServiceServer stop_motor_service = nh.advertiseService("stop_motor", stop_motor);
   // ros::ServiceServer start_motor_service = nh.advertiseService("start_motor", start_motor);

    if(drv->startScan() != Success)
    {
      printf("start failed\n");
      return -1;
    }
    scanLoop(scan_pub,inverted,frame_id);
    
    // done!
   // drv->stop();
    drv->disconnect ();
        ros::spinOnce();
    return 0;
}
