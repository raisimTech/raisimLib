//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_INCLUDE_RAISIM_SERVER_CHARTS_HPP_
#define RAISIM_INCLUDE_RAISIM_SERVER_CHARTS_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include "SerializationHelper.hpp"

namespace raisim {

class Chart {
  friend class raisim::RaisimServer;

 public:
  virtual char* initialize(char* data) = 0;
  virtual char* serialize(char* data) = 0;

 protected:
  std::string title_;
  enum class Type : int32_t {
    TIME_SERIES = 0,
    BAR_CHART
  } type_;

  uint32_t visualTag = 0;
 public:
  Type getType() { return type_; }
};

class TimeSeriesGraph : public Chart {
  friend class raisim::RaisimServer;

 protected:
   // You should create time series graph using RaisimServer
  TimeSeriesGraph(std::string title, std::vector<std::string> names, std::string xAxis, std::string yAxis) :
      size_(int32_t(names.size())), xAxis_(std::move(xAxis)), yAxis_(std::move(yAxis)), names_(std::move(names)) {
    title_ = std::move(title);
    type_ = Type::TIME_SERIES;
  }

 public:
  /**
    * Please read the "atlas" example to see how it works.
    * @param[in] time x coordinate for the following data
    * @param[in] d y coordinates of the data. The order is given when the chart is created */
  void addDataPoints(double time, const raisim::VecDyn& d) {
    RSFATAL_IF(size_ != d.n, "Dimension mismatch. The chart has " << size_ << " categories and the inserted data is " << d.n << "dimension");
    {
      std::lock_guard<std::mutex> guard(mtx_);
      timeStamp_.push(time);
      data_.push(d);

      /// keep the data buffer below than maximum allowed
      if (timeStamp_.size() > BUFFER_SIZE) {
        timeStamp_.pop();
        data_.pop();
      }
    }
  }

protected:
  // not for users //
  void clearData() {
    timeStamp_ = std::queue<double>();
    data_ = std::queue<raisim::VecDyn>();
  }

  char* initialize(char* data) final {
    using namespace server;
    return set(data, title_, names_, xAxis_, yAxis_);
  }

  char* serialize(char* data) final {
    using namespace server;
    std::lock_guard<std::mutex> guard(mtx_);
    data = set(data, (int32_t)timeStamp_.size( ));
    while (!timeStamp_.empty()) {
      data = set(data, timeStamp_.front());
      timeStamp_.pop();
    }
    data = set(data, (int32_t)data_.size( ));
    while (!data_.empty()) {
      data = set(data, data_.front());
      data_.pop();
    }
    clearData();
    return data;
  }

  [[nodiscard]] int32_t size() const { return size_; }

 private:
  int32_t size_;
  std::mutex mtx_;
  std::string xAxis_, yAxis_;
  std::vector<std::string> names_;
  std::queue<raisim::VecDyn> data_;
  std::queue<double> timeStamp_;
  constexpr static int BUFFER_SIZE = 500;
};

class BarChart : public Chart {
  friend class raisim::RaisimServer;

 protected:
  BarChart(std::string title, std::vector<std::string> names) :
      size_(int32_t(names.size())), names_(std::move(names)) {
    title_ = std::move(title);
    type_ = Type::BAR_CHART;
  }

 public:

 /**
  * Please read the "atlas" example to see how it works.
  * @param[in] data values of each category. The histogram will be normalized. */
  void setData(const std::vector<float>& data) {
    RSFATAL_IF(size_ != data.size(), "Dimension mismatch. The chart has " << size_ << " categories and the inserted data is " << data.size() << "dimension");
    {
      std::lock_guard<std::mutex> guard(mtx_);
      data_ = data;
    }
  }

 protected:
  char* initialize(char* data) final {
    using namespace server;
    return set(data, title_, names_);
  }

  char* serialize(char* data) final {
    using namespace server;
    std::lock_guard<std::mutex> guard(mtx_);
    return set(data, data_);
  }

 private:
  int32_t size_;
  std::mutex mtx_;
  std::vector<std::string> names_;
  std::vector<float> data_;

};

}

#endif //RAISIM_INCLUDE_RAISIM_SERVER_CHARTS_HPP_
