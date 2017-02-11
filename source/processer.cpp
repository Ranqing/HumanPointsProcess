#include "processer.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>

PointcloudProcesser::PointcloudProcesser(): m_is_loaded(false), m_cloud(new QingPointcloud()), m_cloud_filtered(new QingPointcloud())
{
}

PointcloudProcesser::~PointcloudProcesser() {

}

void PointcloudProcesser::load_ply(const string filename) {
    cout << "loading " << filename << endl;
    pcl::PLYReader reader;
    reader.read(filename, *m_cloud);

    int size = m_cloud->width * m_cloud->height;
    if(0==size) { m_is_loaded = false; }
    else {m_is_loaded = true;}

    cout << "Pointcloud size: " << m_cloud->width * m_cloud->height
         << " data points (" << pcl::getFieldsList(*m_cloud) << "). " << endl;
}

void PointcloudProcesser::save_ply(const string outfile) {
    pcl::PLYWriter writer;
    writer.write(outfile, *m_cloud_filtered);
    cout << "saving " << outfile << endl;
}

void PointcloudProcesser::down_sampling(const string outfile) {
    m_cloud_filtered = m_cloud->makeShared();
    int src_size = m_cloud->width * m_cloud->height;
    int dst_size = m_cloud_filtered->width * m_cloud_filtered->height;

    //cout << src_size << ", " << dst_size << endl;

    float leafsize = 0.0f, stepsize = 0.05f;

    pcl::VoxelGrid<QingPoint> sor;
    sor.setInputCloud(m_cloud);
    while(dst_size == src_size) {
        leafsize += stepsize;
        sor.setLeafSize(leafsize, leafsize, leafsize);
        sor.filter(*m_cloud_filtered);
        dst_size = m_cloud_filtered->width * m_cloud_filtered->height;
        cout << "after down-sampling, pointcloud size: " << dst_size << ", leafsize = " << leafsize << endl;
    }

    m_cloud = m_cloud_filtered->makeShared();
    src_size = m_cloud->width * m_cloud->height;
    dst_size = m_cloud_filtered->width * m_cloud_filtered->height;
    sor.setInputCloud(m_cloud);
    while(dst_size * 2 > src_size) {
        leafsize += stepsize;
        sor.setLeafSize(leafsize, leafsize, leafsize);
        sor.filter(*m_cloud_filtered);
        dst_size = m_cloud_filtered->width * m_cloud_filtered->height;
        cout << "after down-sampling, pointcloud size: " << dst_size << ", leafsize = " << leafsize << endl;
    }
#if 1
    save_ply(outfile);
# endif
    m_cloud = m_cloud_filtered->makeShared();
}

void PointcloudProcesser::outliers_removal(const string outfile) {
    int numk = 30;
    double stddev = 1.0;

    pcl::StatisticalOutlierRemoval<QingPoint> sor;
    sor.setInputCloud(m_cloud);
    sor.setMeanK(numk);
    sor.setStddevMulThresh (stddev);
    sor.filter(*m_cloud_filtered);

    cout << "after outliers removal: " << *m_cloud_filtered << std::endl;
# if 1
    save_ply(outfile);
# endif
    m_cloud = m_cloud_filtered->makeShared();
}

void PointcloudProcesser::mls_resample(const string outfile) {
    double radius = 0.6;

    pcl::search::KdTree<QingPoint>::Ptr tree(new pcl::search::KdTree<QingPoint>());
    pcl::MovingLeastSquares<QingPoint, QingPoint> mls;

    mls.setComputeNormals(true);
    mls.setInputCloud(m_cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.process(*m_cloud_filtered);

    cout << "after msl_resample: " << *m_cloud_filtered << endl;
# if 1
    save_ply(outfile);
# endif
    m_cloud = m_cloud_filtered->makeShared();
}
