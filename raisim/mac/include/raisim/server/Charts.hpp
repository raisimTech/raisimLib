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
 public:
  virtual char* initialize(char* data) = 0;
  virtual char* serialize(char* data) = 0;

 protected:
  std::string title_;
  enum class Type : int32_t {
    TIME_SERIES = 0,
    BAR_CHART
  } type_;

 public:
  Type getType() { return type_; }
};

class TimeSeriesGraph : public Chart {
 public:
  TimeSeriesGraph(std::string title, std::vector<std::string> names, std::string xAxis, std::string yAxis) :
      size_(names.size()), xAxis_(std::move(xAxis)), yAxis_(std::move(yAxis)), names_(std::move(names)) {
    title_ = std::move(title);
    type_ = Type::TIME_SERIES;
  }

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
    data = set(data, (uint64_t)timeStamp_.size( ));
    while (!timeStamp_.empty()) {
      data = set(data, timeStamp_.front());
      timeStamp_.pop();
    }
    data = set(data, (uint64_t)data_.size( ));
    while (!data_.empty()) {
      data = set(data, data_.front());
      data_.pop();
    }
    clearData();
    return data;
  }

  [[nodiscard]] uint64_t size() const { return size_; }

 private:
  uint64_t size_;
  std::mutex mtx_;
  std::string xAxis_, yAxis_;
  std::vector<std::string> names_;
  std::queue<raisim::VecDyn> data_;
  std::queue<double> timeStamp_;
  constexpr static int BUFFER_SIZE = 500;
};

class BarChart : public Chart {
 public:
  BarChart(std::string title, std::vector<std::string> names) :
      size_(names.size()), names_(std::move(names)) {
    title_ = std::move(title);
    type_ = Type::BAR_CHART;
  }

  void setData(const std::vector<float>& data) {
    RSFATAL_IF(size_ != data.size(), "Dimension mismatch. The chart has " << size_ << " categories and the inserted data is " << data.size() << "dimension");
    {
      std::lock_guard<std::mutex> guard(mtx_);
      data_ = data;
    }
  }

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
  uint64_t size_;
  std::mutex mtx_;
  std::vector<std::string> names_;
  std::vector<float> data_;

};

}

#endif //RAISIM_INCLUDE_RAISIM_SERVER_CHARTS_HPP_