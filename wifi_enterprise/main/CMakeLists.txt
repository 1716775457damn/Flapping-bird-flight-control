# 注册组件并添加所需源文件
idf_component_register(SRCS "car_control_main.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES "web_pages.html")
