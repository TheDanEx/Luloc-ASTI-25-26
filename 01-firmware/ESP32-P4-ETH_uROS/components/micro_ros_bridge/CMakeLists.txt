set(MICRO_ROS_LIB "${COMPONENT_DIR}/libmicroros.a")

if(EXISTS "${MICRO_ROS_LIB}")
    set(COMPONENT_SRCS "")
    set(COMPONENT_REQS nvs_flash esp_eth lwip esp_netif esp_driver_gpio esp_event hal)

    if(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        list(APPEND COMPONENT_SRCS "network_interfaces/uros_ethernet_netif.c")
    endif()

    if(CONFIG_MICRO_ROS_ESP_NETIF_WLAN)
        list(APPEND COMPONENT_SRCS "network_interfaces/uros_wlan_netif.c")
        list(APPEND COMPONENT_REQS esp_wifi)
    endif()

    idf_component_register(
        SRCS "${COMPONENT_SRCS}"
        INCLUDE_DIRS "include" "network_interfaces"
        KCONFIG "${COMPONENT_DIR}/Kconfig.projbuild"
        REQUIRES ${COMPONENT_REQS}
    )

    add_prebuilt_library(microros_prebuilt "${MICRO_ROS_LIB}" REQUIRES lwip)

    target_link_libraries(${COMPONENT_LIB} PUBLIC microros_prebuilt)

    file(GLOB include_folders LIST_DIRECTORIES true CONFIGURE_DEPENDS ${COMPONENT_DIR}/include/*)
    foreach(include_folder ${include_folders})
        if(IS_DIRECTORY ${include_folder})
            get_filename_component(folder_name ${include_folder} NAME)
            target_include_directories(${COMPONENT_LIB} INTERFACE ${COMPONENT_DIR}/include/${folder_name})
        endif()
    endforeach()
else()
    message(FATAL_ERROR "Micro-ROS library not found at ${MICRO_ROS_LIB}.")
endif()
