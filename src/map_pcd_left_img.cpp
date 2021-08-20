#include <iostream>
#include <assert.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// pcl
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>


//octomap 
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// boost.format 字符串处理
#include <boost/format.hpp>


// 使用Eigen的Geometry模块处理3d运动
#include <Eigen/Core>
#include <Eigen/Geometry> 
using namespace std;
using namespace Eigen;
using namespace cv;

float camera_scale  = 1000;
float cx     = 620.5;
float cy     = 367.5;
float fx     = 481.0;
float fy     = 479.0;




Vec3b get_color(cv::Mat image,Eigen::Vector2d pixel){


return image.at<Vec3b>(Point(int(pixel[0]),int(pixel[1])));

// for better color, maybe interpolation?

}

int main( int argc, char** argv )
{
    if (argc != 4)
    {
        cout<<"Usage: map_pcd_left_img <file.pcd> <file.png>  xyzq.txt"<<endl;
        return -1;
    }

    string pcd_path = argv[1], img_path = argv[2],transform_path = argv[3];
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZI> ( pcd_path, cloud );

    cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<endl;

    cv::Mat img = cv::imread(img_path);
    cv::Mat img_fuse = img.clone();
   // cv::imshow("",img);
   // cv::waitKey(0);


//  create the transformation -0.02 -0.17 0.1 0 0 0 1
    ifstream fin;
    fin.open( transform_path );
   // Eigen::Quaterniond q;
   // Eigen::Isometry3d t;
    vector< Eigen::Isometry3d > poses;
   // while( fin.peek() != EOF )
    {
        int index_keyframe;
        float data[7]; // 三个位置加一个姿态四元数x,y,z, w,ux,uy,uz
        fin>>index_keyframe;
        for ( int i=0; i<7; i++ )
        {
            fin>>data[i];
            cout<<data[i]<<" ";
        }
        cout<<endl;
        //if (fin.fail()) break;
        // 注意这里的顺序。g2o文件四元数按 qx, qy, qz, qw来存
        // 但Eigen初始化按照qw, qx, qy, qz来做
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d t(q);
        t(0,3) = data[0]; t(1,3) = data[1]; t(2,3) = data[2];
        poses.push_back(t);
	poses[0] = poses[0];

 
    }
 
//transform the point cloud so as to it is in the left cam frame

cout<<poses[0].matrix() <<endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr( new     pcl::PointCloud<pcl::PointXYZI>() );
    pcl::transformPointCloud( cloud, *trans_cloud_ptr, poses[0].matrix());




 






    //声明octomap变量
    cout<<"copy data into octomap..."<<endl;
    // 创建带颜色的八叉树对象，参数为分辨率，这里设成了0.05
    octomap::ColorOcTree tree( 0.005 );

    for (auto p:trans_cloud_ptr->points)
    {

// construct octomap when there are visual content.
		   
		//
		   double x = fx*p.x/p.z+cx;
		   double y = fy*p.y/p.z+cy;
		   Eigen::Vector2d pixel(x,y); //1280 720
		 

			if(pixel[0]>0 && pixel[1]>0 && pixel[0]<1280 && pixel[1]<720){

		   	cout<<"pcd: "<<p.x<<" "<<p.y<<" "<<p.z<<" pixel: "<<pixel[0]<<" "<<pixel[1]<<endl;
			//cout<<p.intensity<<endl;

			img_fuse.at<Vec3b>(Point(int(pixel[0]),int(pixel[1]))) = 250,0,0;//p.intensity,p.intensity,p.intensity;


			// 将点云里的点插入到octomap中
			tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
			Vec3b color = get_color(img,pixel); 
			tree.integrateNodeColor( p.x, p.y, p.z, color[0],color[1],color[2] );


			}
 
    }


    cv::imshow("",img_fuse);
    cv::waitKey();
    cv::imwrite("fused_img.png",img_fuse);

 
    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap, 注意要存成.ot文件而非.bt文件
    tree.write( "color_coded.ot" );
    cout<<"done."<<endl;

    return 0;
}
