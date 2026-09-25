idf_component_register(SRCS "main.c" "controller_diag.c" "trial_pump.c" "auto_link.c" "llcp_pool.c" "raw_llcp.c" "air_frame.c" "air_feedback.c" "air_event.c" "radio_program.c" "radio_activity.c" "radio_tx.c" "tone_source.c" "air_session.c" "phy_packet.c" "tone_packet.c" "tone_trial.c" "audio_source.c"
    INCLUDE_DIRS "." "../private" LDFRAGMENTS "bridge_crypto.lf" REQUIRES bt nvs_flash esp_timer mbedtls ull_usb esp_driver_spi esp_driver_gpio)
target_sources(${COMPONENT_LIB} PRIVATE "pcm_stream.c" "control_log.c" "parent_ack.c" "status_probe.c")
target_compile_definitions(${COMPONENT_LIB} PRIVATE ULL_TRIAL_TRACE_CAPACITY=4096 ULL_LIVE_USB=1 ULL_AUDIO_REPEAT=0)
# Link the dormant transmitter now so unresolved ROM calls and its ISR placement
# are checked before enabling the E2/tone caller. This does not invoke it.
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-u ull_radio_tx_submit" "-u ull_radio_tx_cancel" "-u ull_radio_tx_collect"
    "-u ull_radio_pdu_submit"
    "-u ull_raw_prepare_session" "-u ull_raw_commit_session"
    "-u ull_tone_packet_build"
    "-u ull_audio_packet_build" "-u ull_air_feedback_accept_frame"
    "-u ull_tone_trial_set_audio_source" "-u ull_audio_source_init" "-u ull_audio_source_push"
    "-u ull_tone_build" "-u ull_air_session_plan_build" "-u ull_air_session_time"
    "-u ull_phy_packet_build" "-u ull_phy_dtm_start")
