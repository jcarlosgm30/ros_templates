#include "ros_pkg_template/algorithm.hpp"

#include <utility>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/count.hpp>


using namespace boost::accumulators;

struct Algorithm::Data {
  accumulator_set<double, features<tag::mean, tag::count>> acc;
};

Algorithm::Algorithm() {
  data_ = std::make_unique<Data>();
}

Algorithm::~Algorithm() = default;

void Algorithm::accReset()
{
  data_->acc = {};
}

void Algorithm::addData(double data)
{
  data_->acc(data);
}

void Algorithm::addData(const Eigen::VectorXd& data)
{
  for(auto i = 0; i < data.size(); ++i)
    addData(data[i]);
}

double Algorithm::getAverage() const
{
  return count(data_->acc) ? mean(data_->acc) : 0;
}

double Algorithm::getDataSize() const
{
    return count(data_->acc);
}
