//#include "wavetable.hpp"
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>

#include "logger.h"
#include "fifo.h"
#include "filter-taps.h"


namespace po = boost::program_options;

// create a block type which is a pair of block number and vector of samples
template <typename samp_type>
using Block = typename std::pair<int, std::vector<samp_type>>;


/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int) {
    stop_signal_called = true;
}



/***********************************************************************
 * Power averaging thread function
 **********************************************************************/
template <typename samp_type>
void power_average(tsFIFO<Block<samp_type>>& fifo_out, const int thread_number) {
    // create logger
    Logger proc_logger("PowerAverage " + std::to_string(thread_number) , "./power_average" + std::to_string(thread_number) + ".log");

    // create dummy block and average variables
    Block<samp_type> block;
    int block_size = 0;
    float average_power = 0;
    float average_power_db = 0;

    // check ctrl-c and fifo empty
    while (not stop_signal_called) {
        if (fifo_out.size() != 0) {
            // pop block from fifo
            fifo_out.pop(block);
            block_size = block.second.size();
            // compute average power
            for (int i=0; i<block_size; i++) {
                average_power += std::pow(std::abs(block.second[i]), 2);
                //proc_logger.log("Sample: " + std::to_string(i) +
                //        "\tAmp: " + std::to_string(std::abs(block.second[i])) +
                //        "\tAve: " + std::to_string(average_power));

            }
            average_power /= block_size;
            average_power_db = 10*std::log10(average_power);
            // print out average
            proc_logger.log("Thread: " + std::to_string(thread_number) +
                            "\tBlock #: " + std::to_string(block.first) +
                            "\tAverage power: " + std::to_string(average_power_db) + "dB");
        }
    }

    // notify user that processing thread is done
    proc_logger.log("Power averaging thread is done and closing");
}



/***********************************************************************
 * Filtering thread function
 **********************************************************************/
template <typename samp_type>
void filter(int D, int U,
        tsFIFO<Block<samp_type>>& fifo_in,
        tsFIFO<Block<samp_type>>& fifo_out) {
    // create logger
    Logger proc_logger("Filter", "./filter.log");
    
    // print out down sampling and up sampling factors
    proc_logger.log("Down-sampling factor D: " + std::to_string(D));
    proc_logger.log("Up-sampling factor U: " + std::to_string(U));

    // create dummy block and average variables
    Block<samp_type> block;
    int block_size = 0;

    // check ctrl-c and fifo empty
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // pop block from fifo
            fifo_in.pop(block);
            block_size = block.second.size();
        } 
    }
    // notify user that processing thread is done
    proc_logger.log("Filtering thread is done and closing");
}



/***********************************************************************
 * Utilities
 **********************************************************************/
//! Change to filename, e.g. from usrp_samples.dat to usrp_samples.00.dat,
//  but only if multiple names are to be generated.
std::string generate_out_filename(
    const std::string& base_fn, size_t n_names, size_t this_name)
{
    if (n_names == 1) {
        return base_fn;
    }

    boost::filesystem::path base_fn_fp(base_fn);
    base_fn_fp.replace_extension(boost::filesystem::path(
        str(boost::format("%02d%s") % this_name % base_fn_fp.extension().string())));
    return base_fn_fp.string();
}


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
        //recv_logger.log("Total number of samles: " + std::to_string(num_total_samps)); 
        //recv_logger.log("Number of rx samples: " + std::to_string(num_rx_samps));
        for (size_t i = 0; i < outfiles.size(); i++) {
            //outfiles[i]->write(
            //    (const char*)buff_ptrs[i], num_rx_samps * sizeof(samp_type));
            // update block before pushing to fifo 
            block.second = buffs[i];
            // push current samples to fifo
            fifo_in.push(block);
            // increment block counter
            block.first++;
        }
        //recv_logger.log("Check fifo size: " + std::to_string(process_fifo.size()));
    }

    // Check fifo entry size
    //process_fifo.pop(block);
    //recv_logger.log("Check fifo block number: " + std::to_string(block.first));
    //recv_logger.log("Check fifo block size: " + std::to_string(block.second.size()));

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


/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // create logger
    Logger main_logger("Main", "./main.log");
    
#ifdef USE_VOLK
    main_logger.log("Using VOLK");
#endif

    // receive variables to be set by po
    std::string ref, otw;
    std::string rx_args, file, type, rx_ant, rx_subdev, rx_channels;
    size_t total_num_samps, rx_spb;
    double rx_rate, rx_freq, rx_gain, rx_bw;
    double settling;
    int D, U;
    std::string taps_filename;

    // program specific variables
    size_t num_proc_threads;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("rx-args", po::value<std::string>(&rx_args)->default_value(""), "uhd receive device address args")
        ("file", po::value<std::string>(&file)->default_value("usrp_samples.dat"), "name of the file to write binary samples to")
        ("type", po::value<std::string>(&type)->default_value("float"), "sample type in file: double, float, or short")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("settling", po::value<double>(&settling)->default_value(double(0.2)), "settling time (seconds) before receiving")
        ("rx-spb", po::value<size_t>(&rx_spb)->default_value(10000), "samples per buffer")
        ("rx-rate", po::value<double>(&rx_rate)->default_value(1000000), "rate of receive incoming samples")
        ("rx-freq", po::value<double>(&rx_freq)->default_value(2437000000), "receive RF center frequency in Hz")
        ("rx-gain", po::value<double>(&rx_gain), "gain for the receive RF chain")
        ("rx-ant", po::value<std::string>(&rx_ant), "receive antenna selection")
        ("rx-subdev", po::value<std::string>(&rx_subdev), "receive subdevice specification")
        ("rx-bw", po::value<double>(&rx_bw), "analog receive filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external, mimo)")
        ("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode")
        ("rx-channels", po::value<std::string>(&rx_channels)->default_value("0"), "which RX channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("rx-int-n", "tune USRP RX with integer-N tuning")
        ("D,d", po::value<int>(&D)->default_value(1), "Down-sampling factor")
        ("U,u", po::value<int>(&U)->default_value(1), "Up-sampling factor")
        ("taps-file", po::value<std::string>(&taps_filename), "filepath of filter taps file")
        ("nthreads", po::value<size_t>(&num_proc_threads)->default_value(1), "number of processing (power averager) threads")
    ;

    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD RX Average Sample Power %s") % desc << std::endl;
        return ~0;
    }

    // setup the filter taps object
    if (not vm.count("taps-file")) {
            std::cerr << "Please specify the filter taps file with --taps-file"
                      << std::endl;
            return ~0;
    }
    FilterTaps taps (taps_filename);   
    main_logger.log("Filter length L: " + std::to_string(taps.len()));

    // create a usrp device
    main_logger.log("Createing the receive usrp device with: " + rx_args);
    uhd::usrp::multi_usrp::sptr rx_usrp = uhd::usrp::multi_usrp::make(rx_args);

    if (vm.count("rx-subdev"))
        rx_usrp->set_rx_subdev_spec(rx_subdev);

    std::vector<std::string> rx_channel_strings;
    std::vector<size_t> rx_channel_nums;
    boost::split(rx_channel_strings, rx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < rx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(rx_channel_strings[ch]);
        if (chan >= rx_usrp->get_rx_num_channels()) {
            throw std::runtime_error("Invalid RX channel(s) specified.");
        } else
            rx_channel_nums.push_back(std::stoi(rx_channel_strings[ch]));
    }

    // Lock mboard clocks
    if (vm.count("ref")) {
        rx_usrp->set_clock_source(ref);
    }
    std::cout << boost::format("Using RX Device: %s") % rx_usrp->get_pp_string()
              << std::endl;
    main_logger.log("Using RX Device: " + rx_usrp->get_pp_string());
    if (not vm.count("rx-rate")) {
        std::cerr << "Please specify the sample rate with --rx-rate" << std::endl;
        return ~0;
    }
    main_logger.log("Setting RX Rate: " + std::to_string(rx_rate/1e6) + "Msps");
    rx_usrp->set_rx_rate(rx_rate);
    main_logger.log("Actual RX Rate: " + std::to_string(rx_usrp->get_rx_rate()/1e6) + "Msps");


    for (size_t ch = 0; ch < rx_channel_nums.size(); ch++) {
        size_t channel = rx_channel_nums[ch];
        if (rx_channel_nums.size() > 1) {
            main_logger.log("Configuring RX Channel " + std::to_string(channel));
        }

        // set the receive center frequency
        if (not vm.count("rx-freq")) {
            std::cerr << "Please specify the center frequency with --rx-freq"
                      << std::endl;
            return ~0;
        }
        main_logger.log("Setting RX Freq: " + std::to_string(rx_freq/1e6) + "MHz");
        uhd::tune_request_t rx_tune_request(rx_freq);
        if (vm.count("rx-int-n"))
            rx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        rx_usrp->set_rx_freq(rx_tune_request, channel);
        main_logger.log("Actual RX Freq: " + std::to_string(rx_usrp->get_rx_freq(channel)/1e6));
        // set the receive rf gain
        if (vm.count("rx-gain")) {
            main_logger.log("Setting RX Gain: " + std::to_string(rx_gain) + "dB");
            rx_usrp->set_rx_gain(rx_gain, channel);
            main_logger.log("Actual RX Gain: " + std::to_string(rx_usrp->get_rx_gain(channel)) + "dB");
        }

        // set the receive analog frontend filter bandwidth
        if (vm.count("rx-bw")) {
            main_logger.log("Setting RX Bandwidth: " + std::to_string(rx_bw/1e6) + "MHz");
            rx_usrp->set_rx_bandwidth(rx_bw, channel);
            main_logger.log("Actual RX Bandwidth: " + std::to_string(rx_usrp->get_rx_bandwidth(channel)/1e6) + "MHz");
        }

        // set the receive antenna
        if (vm.count("rx-ant"))
            rx_usrp->set_rx_antenna(rx_ant, channel);
    }


    // check ref and lo lock detect
    std::vector<std::string> rx_sensor_names;
    rx_sensor_names = rx_usrp->get_rx_sensor_names(0);
    if (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "lo_locked")
        != rx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = rx_usrp->get_rx_sensor("lo_locked", 0);
        main_logger.log("Checking RX: " + lo_locked.to_pp_string());
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }

    rx_sensor_names = rx_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "mimo_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = rx_usrp->get_mboard_sensor("mimo_locked", 0);
        main_logger.log("Checking RX: " + mimo_locked.to_pp_string());
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "ref_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = rx_usrp->get_mboard_sensor("ref_locked", 0);
        main_logger.log("Checking RX: " + ref_locked.to_pp_string());
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    if (total_num_samps == 0) {
        std::signal(SIGINT, &sig_int_handler);
        main_logger.log("Press Ctrl+C to stop streaming");
    }

    // set up threads worker
    std::thread filter_worker;
    std::thread power_average_worker[num_proc_threads];

    // reset usrp time to prepare for transmit/receive
    main_logger.log("Setting device timestamp to 0");
    rx_usrp->set_time_now(uhd::time_spec_t(0.0)); 

    // recv to file as function 
    if (type == "double") {
        // create a fifo buffer for processing
        tsFIFO<Block<std::complex<double>>> fifo_in;
        tsFIFO<Block<std::complex<double>>> fifo_out;
        // create thread for multirate filtering
        filter_worker = std::thread(&filter<std::complex<double>>,
                D, U, std::ref(fifo_in), std::ref(fifo_out));
        // create thread for the power averaging function before receiving samples
        for (int i=0; i<num_proc_threads; i++)
            power_average_worker[i] = std::thread(&power_average<std::complex<double>>,
                    std::ref(fifo_out), i);
        // call receive function
        recv_to_file<std::complex<double>>(
            rx_usrp, "fc64", otw, file,
            rx_spb, total_num_samps, settling,
            rx_channel_nums, fifo_in);
    } else if (type == "float") {
        // create a fifo buffer for processing
        tsFIFO<Block<std::complex<float>>> fifo_in;
        tsFIFO<Block<std::complex<float>>> fifo_out;
        // create thread for multirate filtering
        filter_worker = std::thread(&filter<std::complex<float>>,
                D, U, std::ref(fifo_in), std::ref(fifo_out));
        // create thread for the processing function before receiving samples
        for (int i=0; i<num_proc_threads; i++)
            power_average_worker[i] = std::thread(&power_average<std::complex<float>>,
                    std::ref(fifo_out), i);
        // call receive function
        recv_to_file<std::complex<float>>(
            rx_usrp, "fc32", otw, file,
            rx_spb, total_num_samps, settling,
            rx_channel_nums, fifo_in);
    } else if (type == "short") {
        // create a fifo buffer for processing
        tsFIFO<Block<std::complex<short>>> fifo_in;
        tsFIFO<Block<std::complex<short>>> fifo_out;
        // create thread for multirate filtering
        filter_worker = std::thread(&filter<std::complex<short>>,
                D, U, std::ref(fifo_in), std::ref(fifo_out));
        // create thread for the processing function before receiving samples
        for (int i=0; i<num_proc_threads; i++)
            power_average_worker[i] = std::thread(&power_average<std::complex<short>>,
                    std::ref(fifo_out), i);
        // call receive function
        recv_to_file<std::complex<short>>(
            rx_usrp, "sc16", otw, file,
            rx_spb, total_num_samps, settling,
            rx_channel_nums, fifo_in);
    } else {
        // clean up transmit worker
        stop_signal_called = true;
        //transmit_thread.join_all();
        throw std::runtime_error("Unknown type " + type);
    }
    
    
    // clean up transmit worker
    stop_signal_called = true;
    //transmit_thread.join_all();
    
    // join all processing thread
    if (filter_worker.joinable())
        filter_worker.join();
    for (int i=0; i<num_proc_threads; i++) {
        if (power_average_worker[i].joinable())
            power_average_worker[i].join();
    }

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
