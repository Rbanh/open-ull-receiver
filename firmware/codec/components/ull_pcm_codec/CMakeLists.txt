# Add this directory to EXTRA_COMPONENT_DIRS, or relocate PCM_ROOT after copying.
set(PCM_ROOT "${CMAKE_CURRENT_LIST_DIR}")
set(LC3_FLOAT "${PCM_ROOT}/reference/floating_point")
file(GLOB LC3_SOURCES "${LC3_FLOAT}/*.c")
list(REMOVE_ITEM LC3_SOURCES "${LC3_FLOAT}/codec_exe.c" "${LC3_FLOAT}/ccConvert.c")
idf_component_register(
 SRCS "${PCM_ROOT}/component/mic_codec.c" "${PCM_ROOT}/component/pcm_codec.c" "${PCM_ROOT}/component/resample_2x.c" ${LC3_SOURCES}
 INCLUDE_DIRS "${PCM_ROOT}/component/include"
 PRIV_INCLUDE_DIRS "${PCM_ROOT}/component" "${LC3_FLOAT}"
 LDFRAGMENTS "codec_hot.lf")
target_compile_options(${COMPONENT_LIB} PRIVATE -O3 -fsingle-precision-constant
 -fno-strict-aliasing -ffunction-sections -fdata-sections -fstack-usage
 -Wno-error=incompatible-pointer-types -Wno-error=maybe-uninitialized)
