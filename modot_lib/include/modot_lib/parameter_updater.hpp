#pragma once

#include <rclcpp/rclcpp.hpp>

namespace modot_lib
{
class ParameterUpdater
{
public:
  ParameterUpdater(const std::shared_ptr<rclcpp::ParameterEventHandler>& parameter_event_handler,
                   const rclcpp::Logger& logger = rclcpp::get_logger("parameter_updater"))
    : parameter_event_handler_(parameter_event_handler), logger_(logger)
  {
  }

  template <class T>
  void addParameter(const std::string& parameter_name, T& parameter_store)
  {
    parameter_callback_handle_.push_back(parameter_event_handler_->add_parameter_callback(
        parameter_name,
        [this, &parameter_store](const rclcpp::Parameter& param) { parameterCallback(param, parameter_store); }));
  }

private:
  template <class T>
  void parameterCallback(const rclcpp::Parameter& param, T& store)
  {
    try
    {
      store = param.get_value<T>();
    }
    catch (const rclcpp::ParameterTypeException& exception)
    {
      RCLCPP_ERROR(logger_, exception.what());
    }
  }

  rclcpp::Logger logger_;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> parameter_callback_handle_;
};
}  // namespace modot_lib