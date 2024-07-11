#include <gtest/gtest.h>
#include "Buffer.hpp"

namespace rosneuro {
    template<typename T>
    class BufferTest : public Buffer<T> {
        public:
            BufferTest(void) : Buffer<T>() {};
            ~BufferTest(void) {};

            bool configure(void) { return true; };
            bool add(const DynamicMatrix<T>& in) { return true; };

            bool configure(const std::string& param_name) { return Buffer<T>::configure(param_name); };
            bool configure(XmlRpc::XmlRpcValue& config) { return Buffer<T>::configure(config); };
        private:
            unsigned int size_;
    };

    class BufferTestSuite : public ::testing::Test {
        public:
            BufferTestSuite() {};
            ~BufferTestSuite() {};
            virtual void SetUp(void) {
                buffer = new BufferTest<float>();
            }
            virtual void TearDown(void) {
                delete buffer;
            }
            BufferTest<float>* buffer;
    };

    template<typename T>
    void fromVectorToXmlRpcValue(const std::vector<T>& vec, XmlRpc::XmlRpcValue& value) {
            value.setSize(vec.size());
            for (size_t i = 0; i < vec.size(); ++i)
                value[i] = vec[i];
    }

    TEST_F(BufferTestSuite, SetMethod) {
        EXPECT_TRUE(buffer->set(10, 10));
        EXPECT_EQ(buffer->rows(), 10);
        EXPECT_EQ(buffer->cols(), 10);
    }

    TEST_F(BufferTestSuite, ResizeMethod) {
        buffer->set(10, 10);
        buffer->resize(20, 20);
        EXPECT_EQ(buffer->rows(), 20);
        EXPECT_EQ(buffer->cols(), 20);
    }

    TEST_F(BufferTestSuite, ClearMethod) {
        buffer->set(10, 10);
        buffer->clear();
        EXPECT_TRUE(std::isnan(buffer->get()(0, 0)));
    }

    TEST_F(BufferTestSuite, IsFullMethod) {
        buffer->set(10, 10);
        EXPECT_FALSE(buffer->isfull());
    }

    TEST_F(BufferTestSuite, TypeMethod) {
        EXPECT_EQ(buffer->type(), "");
    }

    TEST_F(BufferTestSuite, ConfigureXmlRpcValueMethod) {
        XmlRpc::XmlRpcValue config;
        config["name"] = "test";
        config["type"] = "test";

        bool result = buffer->configure(config);
        EXPECT_TRUE(result);

        XmlRpc::XmlRpcValue wrong_config;
        wrong_config["wrong"] = "test";
        EXPECT_FALSE(buffer->configure(wrong_config));
    }

    TEST_F(BufferTestSuite, ConfigureParamNameMethod) {
        ros::NodeHandle nh;

        XmlRpc::XmlRpcValue validConfig;
        validConfig["name"] = "test";
        validConfig["type"] = "test";
        nh.setParam("valid_param", validConfig);

        bool result = buffer->configure("valid_param");
        EXPECT_TRUE(result);

        XmlRpc::XmlRpcValue wrongConfig;
        nh.setParam("wrong_param", wrongConfig);

        EXPECT_FALSE(buffer->configure("wrong_param"));
    }

    TEST_F(BufferTestSuite, GetParamString) {
        buffer->params_["name"] = "buffer_name";
        std::string value;
        EXPECT_TRUE(buffer->getParam("name", value));
        EXPECT_EQ(value, "buffer_name");
    }

    TEST_F(BufferTestSuite, GetParamBool) {
        buffer->params_["flag"] = true;
        bool value;
        EXPECT_TRUE(buffer->getParam("flag", value));
        EXPECT_TRUE(value);
    }

    TEST_F(BufferTestSuite, GetParamDouble) {
        buffer->params_["double"] = 1.1;
        double value;
        EXPECT_TRUE(buffer->getParam("double", value));
        EXPECT_DOUBLE_EQ(value, 1.1);
    }

    TEST_F(BufferTestSuite, GetParamInt) {
        buffer->params_["count"] = 42;
        int value;
        EXPECT_TRUE(buffer->getParam("count", value));
        EXPECT_EQ(value, 42);
    }

    TEST_F(BufferTestSuite, GetParamUnsignedInt) {
        buffer->params_["size"] = 100;
        unsigned int value;
        EXPECT_TRUE(buffer->getParam("size", value));
        EXPECT_EQ(value, 100);
    }

    TEST_F(BufferTestSuite, GetParamVectorDouble) {
        std::vector<double> expected_values = {1.1, 2.2, 3.3};

        XmlRpc::XmlRpcValue xml_rpc_value;
        fromVectorToXmlRpcValue(expected_values, xml_rpc_value);

        buffer->params_["values"] = xml_rpc_value;
        std::vector<double> value;
        EXPECT_TRUE(buffer->getParam("values", value));
        EXPECT_EQ(value, expected_values);
    }

    TEST_F(BufferTestSuite, GetParamVectorString) {
        std::vector<std::string> expected_values = {"abc", "def", "ghi"};

        XmlRpc::XmlRpcValue xml_rpc_value;
        fromVectorToXmlRpcValue(expected_values, xml_rpc_value);

        buffer->params_["strings"] = xml_rpc_value;
        std::vector<std::string> value;
        EXPECT_TRUE(buffer->getParam("strings", value));
        EXPECT_EQ(value, expected_values);
    }

    TEST_F(BufferTestSuite, GetParamXmlRpcValue) {
        XmlRpc::XmlRpcValue expected_value = 42;
        buffer->params_["param"] = expected_value;
        XmlRpc::XmlRpcValue value;
        EXPECT_TRUE(buffer->getParam("param", value));
        EXPECT_EQ(value, expected_value);
    }
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "test_ringbuffer");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}