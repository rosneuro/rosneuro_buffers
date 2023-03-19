#ifndef ROSNEURO_BUFFER_HPP
#define ROSNEURO_BUFFER_HPP

#include <ros/ros.h>
#include <Eigen/Dense>

namespace rosneuro {

template<typename T> using DynamicMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T>
class Buffer {

	public:
		Buffer(void); 		
		virtual ~Buffer(void);

		Buffer(const Buffer&) = delete;
		Buffer& operator=(const Buffer&) = delete;

		virtual bool configure(void) = 0;
		virtual bool add(const DynamicMatrix<T>& in) = 0;
		DynamicMatrix<T> get(void);

		void resize(int rows, int cols);
		void clear(void);
		bool isfull(void);
		int rows(void) const;
		int cols(void) const;
		
		std::string type(void) const;
		std::string name(void) const;
		
	protected:
		bool is_configured_;
		bool is_set_;
		std::string type_;
		std::string name_;
	
	protected:
		DynamicMatrix<T> data_;
	
	public: 
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Configuration stuff - To be moved to friend class
	public: 

		bool getParam(const std::string& name, std::string& value) const;
		bool getParam(const std::string& name, bool& value) const;
		bool getParam(const std::string& name, double& value) const;
		bool getParam(const std::string& name, int& value) const;
		bool getParam(const std::string& name, unsigned  int& value) const;
		bool getParam(const std::string& name, std::vector<double>& value) const;
		bool getParam(const std::string& name, std::vector<std::string>& value) const;
		bool getParam(const std::string& name, XmlRpc::XmlRpcValue& value) const;
		
		bool configure(const std::string& param_name);
		bool configure(XmlRpc::XmlRpcValue& config);

	protected:
		bool loadConfiguration(XmlRpc::XmlRpcValue& config);
		bool setNameAndType(XmlRpc::XmlRpcValue& config);
	protected:
		std::map<std::string, XmlRpc::XmlRpcValue> params_;
	private:
		ros::NodeHandle nh_;

};

template<typename T>
Buffer<T>::Buffer(void) : is_configured_(false), is_set_(false) {}

template<typename T>
Buffer<T>::~Buffer(void) {}

template<typename T>
void Buffer<T>::resize(int rows, int cols) {
	this->data_.resize(rows, cols);
	this->clear();
}

template<typename T>
void Buffer<T>::clear(void) {
	this->data_.fill(static_cast<T>(NAN));
}


template<typename T>
bool Buffer<T>::isfull(void) {
	return !(this->data_.hasNaN());
}

template<typename T>
DynamicMatrix<T> Buffer<T>::get(void) {
	return this->data_;
}

template<typename T>
int Buffer<T>::rows(void) const {
	return this->data_.rows();
}

template<typename T>
int Buffer<T>::cols(void) const {
	return this->data_.cols();
}

template<typename T>
std::string Buffer<T>::type(void) const {
	return this->type_;
}

template<typename T>
std::string Buffer<T>::name(void) const {
	return this->name_;
}

template<typename T>
bool Buffer<T>::setNameAndType(XmlRpc::XmlRpcValue& config) {

	if(config.hasMember("name") == false) {
		ROS_ERROR("Buffer didn't have name defined, this is required");
		return false;
	}
	
	if(config.hasMember("type") == false) {
		ROS_ERROR("Buffer didn't have type defined, this is required");
		return false;
	}

	this->name_ = std::string(config["name"]);
	this->type_ = std::string(config["type"]);
	ROS_DEBUG("Configuring Buffer of type: %s with name %s", this->type_.c_str(), this->name_.c_str());
    
	return true;
}

/*** Configure stuff - To be moved to friend class **/

template<typename T>
bool Buffer<T>::configure(const std::string& param_name) {
	
	bool retval = false;
	XmlRpc::XmlRpcValue config;
	if (!this->nh_.getParam(param_name, config)) {
  		ROS_ERROR("Could not find parameter %s on the server, are you sure that it was pushed up correctly?", param_name.c_str());
		return false;
	}

	retval = this->configure(config);
	return retval;
}

template<typename T>
bool Buffer<T>::configure(XmlRpc::XmlRpcValue& config) {
	if (this->is_configured_) {
		ROS_ERROR("Buffer %s of type %s already being reconfigured", this->name_.c_str(), this->type_.c_str());
	}
	this->is_configured_ = false;
	bool retval = true;
	
	retval = retval && this->loadConfiguration(config);
	retval = retval && this->configure();
	this->is_configured_ = retval;
	return retval;
}

template<typename T>
bool Buffer<T>::loadConfiguration(XmlRpc::XmlRpcValue& config) {
	
	if(config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
		ROS_ERROR("A buffer configuration must be a map with fields name, type, and params");
		return false;
	} 
  
	if (!setNameAndType(config)) {
		return false;
	}

	//check to see if we have parameters in our list
	if(config.hasMember("params")) {
  		
		//get the params map
  		XmlRpc::XmlRpcValue params = config["params"];

		if(params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    		ROS_ERROR("params must be a map");
			return false;
		} else {
    	
			//Load params into map
    		for(XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it) {
      			ROS_DEBUG("Loading param %s", it->first.c_str());
      			this->params_[it->first] = it->second;
			}
		}
	}

	return true;
}

template<typename T>
bool Buffer<T>::getParam(const std::string& name, std::string& value) const {
    
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeString)
		return false;

    auto tmp = it->second;
    value = std::string(tmp);
    return true;
}

template<typename T>
bool Buffer<T>::getParam(const std::string& name, bool& value) const {
	
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;

	if(it->second.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
		return false;

	auto tmp = it->second;
	value = (bool)(tmp);
	return true;
}

template<typename T>
bool Buffer<T>::getParam(const std::string& name, double& value) const {
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;
	
	if(it->second.getType() != XmlRpc::XmlRpcValue::TypeDouble && it->second.getType() != XmlRpc::XmlRpcValue::TypeInt)
		return false;
	
	auto tmp = it->second;
	value = it->second.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(tmp) : (double)(tmp);
	return true;
}

template<typename T>
bool Buffer<T>::getParam(const std::string& name, int& value) const {
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;
	
	if(it->second.getType() != XmlRpc::XmlRpcValue::TypeInt)
		return false;
	
	auto tmp = it->second;
	value = tmp;
	return true;
}

template<typename T>
bool Buffer<T>::getParam(const std::string& name, unsigned int& value) const {
	int signed_value;
	if (!this->getParam(name, signed_value))
		return false;
	if (signed_value < 0)
		return false;
	value = signed_value;
	return true;
}

template<typename T>
bool Buffer<T>::getParam(const std::string& name, std::vector<double>& value) const {
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;
	
	value.clear();
	
	if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
		return false;
	
	XmlRpc::XmlRpcValue double_array = it->second;
	
	for (auto i = 0; i < double_array.size(); ++i){
		if(double_array[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && double_array[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
	    return false;
		}
		
		double double_value = double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(double_array[i]) : (double)(double_array[i]);
		value.push_back(double_value);
	}
	
	return true;
}

template<typename T>
bool Buffer<T>::getParam(const std::string& name, std::vector<std::string>& value) const {
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;
	
	value.clear();
	
	if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
		return false;
	
	XmlRpc::XmlRpcValue string_array = it->second;
	
	for (auto i = 0; i < string_array.size(); ++i) {
		if(string_array[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
	    	return false;
	  	}
	
	  	value.push_back(string_array[i]);
	}
	
	return true;
}

template<typename T>
bool Buffer<T>::getParam(const std::string& name, XmlRpc::XmlRpcValue& value) const {
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;
	
	auto tmp = it->second;
	value = tmp;
	return true;
}


}




#endif
