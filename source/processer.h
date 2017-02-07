#ifndef PROCESSER_H
#define PROCESSER_H

#include "../../../Qing/qing_common.h"

typedef pcl::PointXYZRGBNormal QingPoint;
typedef pcl::PointCloud<QingPoint> QingPointcloud;
typedef pcl::PointCloud<QingPoint>::Ptr QingPointcloudPtr;


class PointcloudProcesser
{
public:
    PointcloudProcesser() ;
    ~PointcloudProcesser() ;

    void load_ply(const string infile);
    void save_std_ply(const string outfile, QingPointcloudPtr cloud);
    void down_sampling(const string outfile);
    void outliers_removal(const string outfile);
    void mls_resample(const string outfile);

    bool get_is_loaded() { return m_is_loaded; }

private:
    bool m_is_loaded;
    QingPointcloudPtr m_cloud;
    QingPointcloudPtr m_cloud_filtered;
};

#endif // PROCESSER_H