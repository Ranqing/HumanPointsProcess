#ifndef CONVERTER_H
#define CONVERTER_H

#include "../../../Qing/qing_common.h"

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> QingRGBNPointcloud;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr QingRGBNPointcloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> QingRGBPointcloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr QingRGBPointcloudPtr;

class PointcloudConverter
{
public:
    PointcloudConverter();
    ~PointcloudConverter();

    void load_src_ply(const string infile);
    void save_dst_ply(const string outfile);
    void convert();

private:
    bool m_is_loaded;
    QingRGBNPointcloudPtr m_src_cloud;
    QingRGBPointcloudPtr m_dst_cloud;
};

#endif // CONVERTER_H
