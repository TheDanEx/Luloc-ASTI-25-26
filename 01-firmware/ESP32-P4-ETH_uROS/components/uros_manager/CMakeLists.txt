idf_component_register(SRCS "uros_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES freertos micro_ros_bridge shared_memory esp_timer)
