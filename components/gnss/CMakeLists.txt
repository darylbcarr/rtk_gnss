if(CONFIG_GNSS_DRIVER_ZED_F9P)
    set(gnss_srcs "gnss_ubx.c")
else()
    set(gnss_srcs "gnss_lc29h.c")
endif()

idf_component_register(SRCS ${gnss_srcs}
                        INCLUDE_DIRS "include"
                        REQUIRES driver esp_timer log)
