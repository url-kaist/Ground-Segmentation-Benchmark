//
// Created by jeewon on 22. 3. 20..
//

#ifndef GSEG_BENCHMARK_DATAHANDLE_H
#define GSEG_BENCHMARK_DATAHANDLE_H

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>


namespace datahandle3d {
    template <typename PointT>
    int load_pcd(std::string pcd_name, boost::shared_ptr< pcl::PointCloud< PointT > > dst){
        if (pcl::io::loadPCDFile<PointT> (pcd_name, *dst) == -1)
        {
            PCL_ERROR ("Couldn't read file!!! \n");
            return (-1);
        }
        std::cout << "Loaded " << dst->size () << " data points from " <<pcd_name<< std::endl;
        return 0;
    }
}
#endif //GSEG_BENCHMARK_DATAHANDLE_H
