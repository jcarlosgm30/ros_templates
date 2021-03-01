#pragma once

#include <memory>

#include <Eigen/Core>

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm
{
 public:
  /*!
   * Constructor.
   */
  Algorithm();

  /*!
   * Destructor.
   */
  virtual ~Algorithm();

  /*!
   * Accumulator reset.
   */
  void accReset();


  /*!
   * Add new measurement data.
   * @param data the new data.
   */
  void addData(const double data);

  /*!
   * Add multiple measurements as once.
   * @param data new data.
   */
  void addData(const Eigen::VectorXd& data);

  /*!
   * Get the computed average of the data.
   * @return the average of the data.
   */
  double getAverage() const;

  /*!
   * Get the number of the data accumulated.
   * @return the number of the data accumulated.
   */
  double getDataSize() const;

 private:
 
  //! Forward declared structure that will contain the data
  struct Data;

  //! Pointer to data (pimpl)
  std::unique_ptr<Data> data_;
};