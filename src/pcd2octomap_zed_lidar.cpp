/*************************************************************************
	> File Name: src/pcd2octomap.cpp
	> Author: Gao Xiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年12月12日 星期六 15时51分45秒
 ************************************************************************/

#include <iostream>
#include <assert.h>
#include <string>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//octomap 
#include <octomap/octomap.h>
using namespace std;

int main( int argc, char** argv )
{
    if (argc != 3)
    {
        cout<<"Usage: pcd2octomap <input_file_zed> <input_file_rslidar>  "<<endl;
        return -1;
    }
    string argv2_string(argv[2]);
    size_t end = argv2_string.find(".pcd"); 
    string input_file_zed = argv[1],input_file_rslidar =argv[2], output_file = argv2_string.substr(0,end)+"_octo_lidar_zed.bt";

    cout<<"input file:"<< input_file_zed<<" "<<input_file_rslidar<<" output_file:"<<output_file<<endl;
    pcl::PointCloud<pcl::PointXYZI> cloud_rslidar;
    pcl::io::loadPCDFile<pcl::PointXYZI> ( input_file_rslidar, cloud_rslidar );

    cout<<"Rslidar point cloud loaded, piont size = "<<cloud_rslidar.points.size()<<endl;


    pcl::PointCloud<pcl::PointXYZRGB> cloud_zed;
    pcl::io::loadPCDFile<pcl::PointXYZRGB> ( input_file_zed, cloud_zed );

    cout<<"ZED point cloud loaded, piont size = "<<cloud_zed.points.size()<<endl;

    //声明octomap变量
    cout<<"copy data into octomap..."<<endl;
    // 创建八叉树对象，参数为分辨率，这里设成了0.05
    octomap::OcTree tree( 0.05 );

    for (auto p:cloud_rslidar.points)
    {
        // 将点云里的点插入到octomap中
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap
   // tree.writeBinary( output_file );
    cout<<"done."<<endl;

    return 0;
}
