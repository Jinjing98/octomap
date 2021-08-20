/*************************************************************************
	> File Name: src/pcd2octomap.cpp
	> Author: Gao Xiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年12月12日 星期六 15时51分45秒
 ************************************************************************/

#include <iostream>
#include <assert.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//octomap 
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
// opencv 用于图像数据读取与处理
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// octomap 
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>

// opencv 用于图像数据读取与处理
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// 使用Eigen的Geometry模块处理3d运动
#include <Eigen/Core>
#include <Eigen/Geometry> 

// pcl
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

// boost.format 字符串处理
#include <boost/format.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;

int main( int argc, char** argv )
{
    if (argc != 3)
    {
        cout<<"Usage: fues_rs_img <input_file.png> <rslidar.pcd>"<<endl;
        return -1;
    }

    string img_file = argv[1], pcd_file = argv[2];
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZI> ( pcd_file, cloud );

    cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<endl;
    
    
    cv::Mat img = cv::imread(img_file);
    
    
    
    float  cx     = 651.5;
    float  cy     = 370.5;
    float  fx     = 534.0;
    float  fy     = 533.0;
    
    
    
    
    
    Eigen::Quaterniond q(1,0,0,0);
    Eigen::Isometry3d T(q);
    T.pretranslate(Vector3d(-0.02  , -0.17,  0.1)); //-0.02   -0.17  0.1
    
    cout<<"the T  mat is: "<<T.matrix()<<endl;
    
    
    
    
        // 将cloud旋转之后插入全局地图
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_trans( new pcl::PointCloud<pcl::PointXYZI>() );
    pcl::transformPointCloud( cloud, *pcd_trans, T.matrix() );
    
    cv::Mat new_img = img.clone();
    
    for (auto p:pcd_trans->points){
        Vector2d pixel = Vector2d(p.y/p.x*fx+cx,p.z/p.x*fy+cy);
        cout<<"the pixel pos is :"<<pixel[0]<<" "<<pixel[1]<<endl;
         
        if (int(p.y/p.x*fx+cx)>0 &&  int(p.y/p.x*fx+cx)<1280  && int(p.z/p.x*fy+cy)>0 && int(p.z/p.x*fy+cy)<720 )
       // img.at<Vec3b>(int(p.y/p.x*fx+cx),int(p.z/p.x*fy+cy)) = 250,0,0;//        img.at<Vec3b>(int(p.z/p.x*fy+cy),int(p.y/p.x*fx+cx))
        
        new_img.at<Vec3b>(int(p.z/p.x*fy+cy),int(p.y/p.x*fx+cx)) = int(p.intensity),0,0;
        
    } 
    cv::imshow("",new_img);
    cv::waitKey();
    cv::imwrite("/home/jinjing/octomap/octomap_tutor_gx/NEW_DATA/fuse.jpeg",new_img);
    
    
    octomap::ColorOcTree tree( 0.05 );
 

    for (auto p:pcd_trans->points)
    {
        // 将点云里的点插入到octomap中
       
       
       
        Vector2d pixel = Vector2d(p.y/p.x*fx+cx,p.z/p.x*fy+cy);
        Vec3b color;
        cout<<"the pixel pos is :"<<pixel[0]<<" "<<pixel[1]<<endl;
         
        if (int(p.y/p.x*fx+cx)>0 &&  int(p.y/p.x*fx+cx)<1280  && int(p.z/p.x*fy+cy)>0 && int(p.z/p.x*fy+cy)<720 ){
            
            tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
            color = img.at<Vec3b>(int(p.z/p.x*fy+cy),int(p.y/p.x*fx+cx)); // bgr
            cout<<"color: "<<color<<endl;
            tree.integrateNodeColor( p.x, p.y, p.z, color[2],color[1],color[0]); //rgb
            
        }
    }
 

 
    tree.updateInnerOccupancy();
    // 存储octomap, 注意要存成.ot文件而非.bt文件
    tree.write( "hhh.ot" );
    cout<<"done."<<endl;

    return 0;
}
