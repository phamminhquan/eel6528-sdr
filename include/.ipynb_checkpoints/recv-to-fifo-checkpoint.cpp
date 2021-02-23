/***********************************************************************
 * recv_to_file function
 **********************************************************************/
template <typename samp_type>
void recv_to_file(uhd::usrp::multi_usrp::sptr usrp,
    const std::string& cpu_format,
    const std::string& wire_format,
    const std::string& file,
    size_t samps_per_buff,
    int num_requested_samples,
    double settling_time,
    std::vector<size_t> rx_channel_nums,
    tsFIFO<Block<samp_type>>& fifo_in)
{
    // create logger
    Logger recv_logger("Recv", "./recv.log");
    
    int num_total_samps = 0;
    // create a receive streamer
    recv_logger.log("Create receive streamer");
    uhd::stream_args_t stream_args(cpu_format, wire_format);
    stream_args.channels             = rx_channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    // allocate a buffer which we re-use for each channel
    if (samps_per_buff == 0)
        samps_per_buff = rx_stream->get_max_num_samps() * 10;
 
    // Prepare buffers for received samples and metadata
    recv_logger.log("Prepare buffers for received samples and metadata");
    recv_logger.log("Number of receive channels: " + std::to_string(rx_channel_nums.size()));
    recv_logger.log("Number of samples per buffer: " + std::to_string(samps_per_buff));
    uhd::rx_metadata_t md;
    std::vector<std::vector<samp_type>> buffs(
        rx_channel_nums.size(), std::vector<samp_type>(samps_per_buff));
    // create a vector of pointers to point to each of the channel buffers
    std::vector<samp_type*> buff_ptrs;
    for (size_t i = 0; i < buffs.size(); i++) {
        buff_ptrs.push_back(&buffs[i].front());
    }

    // Create one ofstream object per channel
    // (use shared_ptr because ofstream is non-copyable)
    recv_logger.log("Create ofstream object");
    std::vector<boost::shared_ptr<std::ofstream>> outfiles;
    for (size_t i = 0; i < buffs.size(); i++) {
        const std::string this_filename = generate_out_filename(file, buffs.size(), i);
        outfiles.push_back(boost::shared_ptr<std::ofstream>(
            new std::ofstream(this_filename.c_str(), std::ofstream::binary)));
    }
    UHD_ASSERT_THROW(outfiles.size() == buffs.size());
    UHD_ASSERT_THROW(buffs.size() == rx_channel_nums.size());
    bool overflow_message = true;
    double timeout =
        settling_time + 0.1f; // expected settling time + padding for first recv

    // setup streaming
    recv_logger.log("Setup streaming");
    uhd::stream_cmd_t stream_cmd((num_requested_samples == 0)
                                     ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS
                                     : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = num_requested_samples;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec  = uhd::time_spec_t(settling_time);
    rx_stream->issue_stream_cmd(stream_cmd);
    
    recv_logger.log("Number of requested samples: " + std::to_string(num_requested_samples));

    recv_logger.log("While loop");
    // create a block to push to fifo and block counter
    Block<samp_type> block;
    block.first = 0;
    while (not stop_signal_called
           and (num_requested_samples > num_total_samps or num_requested_samples == 0)) {
        size_t num_rx_samps = rx_stream->recv(buff_ptrs, samps_per_buff, md, timeout);
        timeout             = 0.1f; // small timeout for subsequent recv

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while streaming") << std::endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            if (overflow_message) {
                overflow_message = false;
                std::cerr
                    << boost::format(
                           "Got an overflow indication. Please consider the following:\n"
                           "  Your write medium must sustain a rate of %fMB/s.\n"
                           "  Dropped samples will not be written to the file.\n"
                           "  Please modify this example for your purposes.\n"
                           "  This message will not appear again.\n")
                           % (usrp->get_rx_rate() * sizeof(samp_type) / 1e6);
            }
            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            throw std::runtime_error(
                str(boost::format("Receiver error %s") % md.strerror()));
        }

        num_total_samps += num_rx_samps;
        for (size_t i = 0; i < outfiles.size(); i++) {
            //outfiles[i]->write(
                //(const char*)buff_ptrs[i], num_rx_samps * sizeof(samp_type));
            //outfiles[i]->write((const char*)buff_ptrs[i], 3*sizeof(samp_type));
            // update block before pushing to fifo 
            block.second = buffs[i];
            // push current samples to fifo
            fifo_in.push(block);
            // increment block counter
            block.first++;
        }
    } 

    // Shut down receiver
    recv_logger.log("Shut down receiver");
    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    rx_stream->issue_stream_cmd(stream_cmd);

    // Close files
    recv_logger.log("Close files");
    for (size_t i = 0; i < outfiles.size(); i++) {
        outfiles[i]->close();
    }
}
