
make_gettext_mofiles(CnoidVnoidMarkerVisualizerPlugin mofiles)
choreonoid_add_plugin(CnoidVnoidMarkerVisualizerPlugin
 marker_visualizer.cpp
 marker_visualizer.h
 )

target_link_libraries(CnoidVnoidMarkerVisualizerPlugin PRIVATE CnoidBodyPlugin)
target_link_libraries(CnoidVnoidMarkerVisualizerPlugin PRIVATE vnoid_lib)
