#include "../LDtc/Detection/DirectMethod.h"
#include "../LDtc/Detection/QuadMethod.h"
#include "../LDtc/Detection/VectorMethod.h"
#include "TestTools.h"

TEST(DIRECT_DETECT, IN_TRIANGLE){
    TIME_POINT_EQ(ld::det::direct_detect({{-5, -5, 15.640312423743286},
                                          {5, -5, 15.640312423743286},
                                          {0, 5, 15.6}},
                                         10),
                  ld::TimePoint<>(0, -1, 15))}

TEST(DIRECT_DETECT, OUT_TRIANGLE_1){
    TIME_POINT_EQ(ld::det::direct_detect({{1, 1, 11.17851130197758},
                                          {10, 2, 10.424918292799399},
                                          {12, 3, 10.25}},
                                         12),
                  ld::TimePoint<>(15, 3, 10))}

TEST(DIRECT_DETECT, OUT_TRIANGLE_2){
    TIME_POINT_EQ(ld::det::direct_detect({{-10, -15, 13.77307714089059},
                                          {-2, -5, 12.927361345041716},
                                          {-8, -20, 14.174159928789397}},
                                         12),
                  ld::TimePoint<>(-5, 30, 10))}

TEST(DIRECT_DETECT, DETECTOR_LINE){
    TIME_POINT_EQ(ld::det::direct_detect({{-1, -1, 4.58113883008419},
                                          {-2, -2, 4.414213562373095},
                                          {-4, -4, 5}},
                                         2),
                  ld::TimePoint<>(NAN, NAN, NAN))}

TEST(DIRECT_DETECT, ON_EDGE){
    TIME_POINT_EQ(ld::det::direct_detect({{-1, 0, 3.5}, {1, 0, 3.5}, {0, 2, 4}},
                                         2),
                  ld::TimePoint<>(0, 0, 3))}

TEST(DIRECT_DETECT, BEHIND_DETECTOR){TIME_POINT_EQ(
    ld::det::direct_detect(
        {{-1, 0, 5.061552812808831}, {1, 0, 5.061552812808831}, {0, 2, 4}}, 2),
    ld::TimePoint<>(NAN, NAN, NAN))}

TEST(DIRECT_DETECT, FOUR_DETECTORS){
    TIME_POINT_EQ(ld::det::direct_detect({{-1, 0, 5.061552812808831},
                                          {1, 0, 5.061552812808831},
                                          {0, 2, 4},
                                          {4, 2, 5.23606797749979}},
                                         2),
                  ld::TimePoint<>(0, 4, 3))}

TEST(VECTOR_DETECT, IN_TRIANGLE){
    TIME_POINT_EQ(ld::det::vector_detect({{-5, -5, 15.640312423743286},
                                          {5, -5, 15.640312423743286},
                                          {0, 5, 15.6}},
                                         10),
                  ld::TimePoint<>(0, -1, 15))}

TEST(VECTOR_DETECT, OUT_TRIANGLE_1){
    TIME_POINT_EQ(ld::det::vector_detect({{1, 1, 11.17851130197758},
                                          {10, 2, 10.424918292799399},
                                          {12, 3, 10.25}},
                                         12),
                  ld::TimePoint<>(15, 3, 10))}

TEST(VECTOR_DETECT, OUT_TRIANGLE_2){
    TIME_POINT_EQ(ld::det::vector_detect({{-10, -15, 13.77307714089059},
                                          {-2, -5, 12.927361345041716},
                                          {-8, -20, 14.174159928789397}},
                                         12),
                  ld::TimePoint<>(-5, 30, 10))}

TEST(VECTOR_DETECT, DETECTOR_LINE){
    TIME_POINT_EQ(ld::det::vector_detect({{-1, -1, 4.58113883008419},
                                          {-2, -2, 4.414213562373095},
                                          {-4, -4, 5}},
                                         2),
                  ld::TimePoint<>(NAN, NAN, NAN))}

TEST(VECTOR_DETECT, ON_EDGE){
    TIME_POINT_EQ(ld::det::vector_detect({{-1, 0, 3.5}, {1, 0, 3.5}, {0, 2, 4}},
                                         2),
                  ld::TimePoint<>(0, 0, 3))}

TEST(VECTOR_DETECT, BEHIND_DETECTOR){TIME_POINT_EQ(
    ld::det::vector_detect(
        {{-1, 0, 5.061552812808831}, {1, 0, 5.061552812808831}, {0, 2, 4}}, 2),
    ld::TimePoint<>(NAN, NAN, NAN))}

TEST(VECTOR_DETECT, FOUR_DETECTORS){
    TIME_POINT_EQ(ld::det::vector_detect({{-1, 0, 5.061552812808831},
                                          {1, 0, 5.061552812808831},
                                          {0, 2, 4},
                                          {4, 2, 5.23606797749979}},
                                         2),
                  ld::TimePoint<>(0, 4, 3))}

TEST(QUAD_DETECT, UNDEFINED){
    TIME_POINT_EQ(ld::det::quad_detect({{-5, -5, 11.781024967590666},
                                        {-5, 5, 11.640312423743286},
                                        {5, 5, 11.640312423743286},
                                        {5, -5, 11.781024967590666}},
                                       10),
                  ld::TimePoint<>(NAN, NAN, NAN))}

TEST(QUAD_DETECT, IN_QUAD) {
    TIME_POINT_EQ(ld::det::quad_detect({{-5, -5, 11.781024967590666},
                                        {-5, 5, 11.640312423743286},
                                        {6, 5, 11.721110255092798},
                                        {6, -5, 11.848528137423857}},
                                       10),
                  ld::TimePoint<>(0, 1, 11));
}

TEST(QUAD_DETECT, OUT_QUAD_1) {
    TIME_POINT_EQ(ld::det::quad_detect({{-3, -4, 5.279508497187473},
                                        {-4, -3, 5.375},
                                        {0, 2, 6.908042190309218},
                                        {5, -4, 7.253469547164993}},
                                       4),
                  ld::TimePoint<>(-4, -4.5, 5));
}

TEST(QUAD_DETECT, OUT_QUAD_2) {
    TIME_POINT_EQ(ld::det::quad_detect({{-3, -4, 8.553167600887974},
                                        {-4, -3, 8.535533905932738},
                                        {0, 2, 6.9525624189766635},
                                        {5, -4, 7.761340254296815}},
                                       4),
                  ld::TimePoint<>(6, 7, 5));
}

TEST(QUAD_DETECT, BEHIND_DETECTOR) {
    TIME_POINT_EQ(ld::det::quad_detect({{-5, -5, 12.555634918610405},
                                        {-5, 5, 12.104536101718725},
                                        {5, 5, 11.14142135623731},
                                        {5, -4, 12.004987562112088}},
                                       10),
                  ld::TimePoint<>(6, 6, 11));
}

TEST(QUAD_DETECT, ON_EDGE) {
    TIME_POINT_EQ(ld::det::quad_detect({{-5, -5, 9.666666666666666},
                                        {-5, 5, 7.666666666666667},
                                        {5, 5, 10.399346342395189},
                                        {5, -4, 11.068851871911235}},
                                       3),
                  ld::TimePoint<>(-5, 3, 7));
}

TEST(QUAD_DETECT, FIVE_DETECTORS) {
    TIME_POINT_EQ(ld::det::quad_detect({{-3, -4, 8.553167600887974},
                                        {-4, -3, 8.535533905932738},
                                        {0, 2, 6.9525624189766635},
                                        {5, -4, 7.761340254296815},
                                        {7, 6, 5.353553390593274}},
                                       4),
                  ld::TimePoint<>(6, 7, 5));
}
