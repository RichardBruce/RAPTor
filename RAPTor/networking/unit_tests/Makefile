# Include project info makefile
include ../Project.mk

TEST_SOURCE = vector_stream_tests.cc msg_header_tests.cc msg_data_tests.cc stack_controller_tests.cc \
	fragmenter_tests.cc batcher_tests.cc drop_late_sequencer_tests.cc reorder_sequencer_tests.cc \
	ack_retransmission_tests.cc nack_retransmission_tests.cc rate_limiter_tests.cc message_delivery_tests.cc \
	udp_connection_tests.cc tcp_connection_tests.cc group_tests.cc \
	$(LIBARYS_PATH)/teamcity/teamcity_boost.cc  $(LIBARYS_PATH)/teamcity/teamcity_messages.cc

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out vector_stream_tests.out msg_header_tests.out msg_data_tests.out stack_controller_tests.out \
	fragmenter_tests.out batcher_tests.out drop_late_sequencer_tests.out reorder_sequencer_tests.out \
	ack_retransmission_tests.out nack_retransmission_tests.out rate_limiter_tests.out message_delivery_tests.out \
	udp_connection_tests.out tcp_connection_tests.out group_tests.out

#$(eval $(call test_suite_template, clipping_tests.out, physics_common.o))
$(eval $(call test_suite_template, ack_retransmission_tests.out, ))
$(eval $(call test_suite_template, batcher_tests.out, ))
$(eval $(call test_suite_template, drop_late_sequencer_tests.out, ))
$(eval $(call test_suite_template, fragmenter_tests.out, ))
$(eval $(call test_suite_template, group_tests.out, ))
$(eval $(call test_suite_template, message_delivery_tests.out, ))
$(eval $(call test_suite_template, msg_data_tests.out, ))
$(eval $(call test_suite_template, msg_header_tests.out, ))
$(eval $(call test_suite_template, nack_retransmission_tests.out, ))
$(eval $(call test_suite_template, rate_limiter_tests.out, ))
$(eval $(call test_suite_template, reorder_sequencer_tests.out, ))
$(eval $(call test_suite_template, stack_controller_tests.out, ))
$(eval $(call test_suite_template, tcp_connection_tests.out, ))
$(eval $(call test_suite_template, udp_connection_tests.out, ))
$(eval $(call test_suite_template, vector_stream_tests.out, ))
