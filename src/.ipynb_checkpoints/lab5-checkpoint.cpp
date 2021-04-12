#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>
#include <boost/crc.hpp>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#include <algorithm>
#include <map>
#include <random>
#include <string>
#include <cstring>
#include <bitset>
#include <ctime>
#include <ratio>
//#include <chrono>


#include "utilities.h"
#include "logger.h"
#include "fifo.h"
#include "filter-taps.h"
#include "filters.hpp"
#include "pulse.h"
#include "block.h"
#include "power-average.h"
#include "stop-signal.h"
#include "fixed-queue.h"
#include "sig-seq.h"
#include "payload.h"
#include "preamble.h"
#include "timer.h"


namespace po = boost::program_options;


/***********************************************************************
 * Preping all the ack packets to be sent out
 **********************************************************************/
void ack_prepare (tsFIFO<Block<bool>>& fifo_out,
                  size_t num_packets)
{
    // create logger
    Logger logger("ACKPrep", "./ack_prep.log");
    // create dummy block
    Block<bool> out_block;
    // packetize the complete fragments
    for (size_t i=1; i<num_packets+1; i++) {
        out_block.first = i;
        logger.logf("Pushing ACK Block: " + std::to_string(out_block.first));
        fifo_out.push(out_block);
    }
    logger.log("Total number of ACKs: " + std::to_string(fifo_out.size()));
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * Sink ARQ scheduler
 **********************************************************************/
void snk_arq_schedule (tsFIFO<Block<bool>>& fifo_in,
                       tsFIFO<Block<std::complex<float>>>& ack_fifo_in,
                       tsFIFO<Block<bool>>& fifo_out,
                       tsFIFO<Block<std::complex<float>>>& ack_fifo_out,
                       size_t payload_size)
{
    // create logger
    Logger logger("ARQ", "./arq.log");
    // create temp blocks
    Block<bool> in_block;
    Block<bool> out_block;
    out_block.second.resize(payload_size);
    Block<std::complex<float>> ack_block;
    // create block counter
    int S = 0;
    int R = 0;
    bool first = true;
    // set up timer
    Timer timer;
    
    while (not stop_signal_called) {
        if (ack_fifo_in.size() != 0) {
            ack_fifo_in.pop(ack_block);
            ack_fifo_out.push(ack_block);
            //logger.log("Sending ACK: " + std::to_string(ack_block.first));
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    //while (not stop_signal_called) {
    //    if (fifo_in.size() != 0) {
    //        // receiving data so clear first flag
    //        first = false;
    //        // pop decoded info block
    //        fifo_in.pop(in_block);
    //        // S is contained in block.first
    //        S = in_block.first;
    //        // check R
    //        if (R == S) {
    //            // send ack first
    //            // increment R
    //            R++;
    //            // push ack
    //            ack_fifo_in.pop(ack_block);
    //            logger.log("Pushing ACK Block: " + std::to_string(ack_block.first));
    //            ack_fifo_out.push(ack_block);
    //            // grab data
    //            out_block = in_block;
    //            fifo_out.push(out_block);
    //            // log for debug
    //            logger.log("S = " + std::to_string(S) + "\tR = " + std::to_string(R));
    //        } else {
    //            // received packet is out of order, resend request
    //            ack_fifo_out.push(ack_block);
    //            // log for debug
    //            logger.log("Error: S = " + std::to_string(S) + "\tR = " + std::to_string(R));
    //        }
    //        // reset timer to now
    //        timer.reset();
    //    } else {
    //        if (!first) { // waiting for first packet should not be time out
    //            // calculate time elapsed from packet transmitted (in seconds)
    //            double timer_count = timer.elapse();
    //            if (timer_count > 5.0) { // more than 2s has elapsed
    //                // resend request at time out
    //                ack_fifo_out.push(ack_block);
    //                // log for debug
    //                logger.log("Time out: " + std::to_string(timer_count) +
    //                           "\tACK FIFO size: " + std::to_string(ack_fifo_out.size()));
    //                timer.reset();
    //            }
    //        }
    //    }
    //}
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * Thread to test source arq scheduler
 **********************************************************************/
void src_arq_schedule_test (tsFIFO<Block<std::complex<float>>>& fifo_in,
                           tsFIFO<Block<bool>>& fifo_out)
{
    // create logger
    Logger logger("ARQTest", "./arq_test.log");
    // create temp blocks
    Block<std::complex<float>> in_block;
    Block<bool> out_block;
    out_block.second.resize(16);
    // create block counter
    int S = 0;
    int R = 0;
    
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // pop ack
            fifo_in.pop(in_block);
            S = in_block.first;
            // check S
            if (S == R) {
                // increment R and make new ack
                R++;
                std::bitset<16> R_bitset (R);
                for (size_t i=0; i<16; i++)
                    out_block.second[i] = R_bitset[i];
                // push new packet
                logger.log("R: " + std::to_string(R) +
                           "\tSize: " + std::to_string(out_block.second.size()));
                // push new packet
                fifo_out.push(out_block);
                // wait a little bit
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
    
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * Source ARQ scheduler
 **********************************************************************/
void src_arq_schedule (tsFIFO<Block<std::complex<float>>>& fifo_in,
                       tsFIFO<Block<std::complex<float>>>& fifo_out,
                       tsFIFO<Block<bool>>& ack_fifo)
{
    // create logger
    Logger logger("ARQ", "./arq.log");
    // create temp blocks
    Block<std::complex<float>> in_block;
    Block<std::complex<float>> out_block;
    Block<bool> ack_block;
    // create block counter
    int S = 0;
    int R = 0;
    bool first = true;
    // set up timer
    Timer timer;
    
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            if (first) {
                // clear first packet flag
                first = false;
                // getting payload from fifo
                fifo_in.pop(in_block);
                out_block = in_block;
                logger.log("First S: " + std::to_string(S) +
                           "\tSize: " + std::to_string(out_block.second.size()));
                // wait for a little bit
                std::this_thread::sleep_for(std::chrono::seconds(1));
                // push to fifo out
                fifo_out.push(out_block);
                // start timer after first packet is pushed
                timer.reset();
            } else {
                if (ack_fifo.size() != 0) {
                    // log to see arrival queue length should be a lot
                    logger.log("ARQ input fifo size: " + std::to_string(fifo_in.size()));
                    // pop ack
                    ack_fifo.pop(ack_block);
                    // check ack content to get R
                    R = ack_block.first;
                    // check R
                    if (R > S) {
                        // push new packet
                        // getting payload from fifo
                        fifo_in.pop(in_block);
                        // create 16-bit packet number bitset
                        S++;
                        out_block = in_block;
                        logger.log("S: " + std::to_string(S) +
                                   "\tSize: " + std::to_string(out_block.second.size()));
                        // push new packet
                        fifo_out.push(out_block);
                    } else {
                        logger.log("ACK R is less than or equal to S");
                        // push same packet as last time
                        fifo_out.push(out_block);
                    }
                    // reset timer to now
                    timer.reset();
                } else {
                    // calculate time elapsed from packet transmitted (in seconds)
                    double timer_count = timer.elapse();
                    if (timer_count > 2.0) { // more than 2s has elapsed
                        logger.log("Time out: " + std::to_string(timer_count));
                        // push same packet as last time
                        fifo_out.push(out_block);
                        timer.reset();
                    }
                }
            }
        }
    }
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * Read in payload file
 **********************************************************************/
void read_payload (std::string filename,
                   tsFIFO<Block<unsigned char>>& fifo_out,
                   tsFIFO<Block<bool>>& temp_fifo_out,
                   size_t frag_size)
{
    // create logger
    Logger logger("ReadPayload", "./read_payload.log");
    logger.log("Payload filename: " + filename);
    logger.log("Fragmentation size: " + std::to_string(frag_size) + " bytes");
    // Define file stream object, and open the file
    std::ifstream file(filename, ios::binary);
    // Prepare iterator pairs to iterate the file content!
    std::istream_iterator<unsigned char> begin(file), end;
    // Reading the file content using the iterator!
    std::vector<unsigned char> buffer(begin,end);
    logger.log("Total payload size: " + std::to_string(buffer.size()) + " bytes");
    // fragment total payload into packets
    Block<unsigned char> out_block;
    out_block.second.resize(frag_size);
    Block<bool> temp_out_block;
    temp_out_block.second.resize(frag_size*8);
    size_t num_frag = std::floor(buffer.size()/frag_size);
    logger.log("Floor number of fraqments: " + std::to_string(num_frag));
    // packetize the complete fragments
    for (size_t i=0; i<num_frag; i++) {
        out_block.first = i;
        temp_out_block.first = out_block.first;
        for (size_t k=0; k<frag_size; k++)
            out_block.second[k] = buffer[i*(frag_size)+k];
        temp_out_block.second = to_bool_vec(out_block.second);
        logger.logf("Pushing fragment: " + std::to_string(out_block.first));
        fifo_out.push(out_block);
        temp_fifo_out.push(temp_out_block);
    }
    // packetize the last remaining bytes
    size_t rem_num_bytes = buffer.size()-num_frag*frag_size;
    if (rem_num_bytes != 0) {
        out_block.first++;
        temp_out_block.first = out_block.first;
        std::fill(out_block.second.begin(), out_block.second.end(), 0);
        for (size_t k=0; k<rem_num_bytes; k++)
            out_block.second[k] = buffer[num_frag*(frag_size)+k];
        temp_out_block.second = to_bool_vec(out_block.second);
        logger.logf("Pushing fragment: " + std::to_string(out_block.first));
        fifo_out.push(out_block);
        temp_fifo_out.push(temp_out_block);
    }
    
    logger.log("Char block size: " + std::to_string(out_block.second.size()));
    logger.log("Bit block size: " + std::to_string(temp_out_block.second.size()));
    logger.log("Char FIFO size: " + std::to_string(fifo_out.size()));
    logger.log("Bit FIFO size: " + std::to_string(temp_fifo_out.size()));
    
    // close output file
    file.close();
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * CRC decode payload
 **********************************************************************/
void ecc_decode (tsFIFO<Block<bool>>& fifo_in,
                 tsFIFO<Block<bool>>& fifo_out,
                 size_t payload_size)
{
    // create logger
    Logger logger("EccDecode", "./ecc_decode.log");
    logger.log("Payload size: " + std::to_string(payload_size));
    // Instantiate a crc-32 object
    boost::crc_32_type crc32;
    // create dummy block
    Block<bool> in_block;
    Block<bool> out_block;
    out_block.second.resize(payload_size-16);
    std::vector<unsigned char> payload_char_vec;
    std::vector<bool> rx_crc_vec;
    std::bitset<32> rx_crc_bitset;
    std::bitset<16> header;
    
    int check_sum_diff = 0;
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // pop input block
            fifo_in.pop(in_block);
            // get the information and ecc part of the payload
            std::vector<bool> payload_vec (in_block.second.begin(),
                                           in_block.second.begin()+payload_size);
            std::vector<bool> rx_crc_vec (in_block.second.begin()+payload_size,
                                          in_block.second.begin()+payload_size+32);
            // convert vector of boolean to vector of unsigned char
            payload_char_vec = to_uchar_vec(payload_vec);
            // convert crc vector of boolean to bitset to uint32
            for (size_t i=0; i<32; i++)
                rx_crc_bitset[i] = rx_crc_vec[i];
            boost::uint32_t rx_crc = rx_crc_bitset.to_ulong();
            // do crc calculation
            crc32.reset();
            crc32.process_bytes(payload_char_vec.data(), payload_char_vec.size());
            // compare checksum
            if (rx_crc == crc32.checksum()) { // checksum are equal
                // separate header and info
                for (size_t i=0; i<16; i++)
                    header[i] = payload_vec[i];
                std::vector<bool> info_vec (payload_vec.begin()+16,
                                            payload_vec.begin()+16+payload_len);
                out_block.first = header.to_ulong();
                out_block.second = info_vec;
                logger.log("Header: " + std::to_string(out_block.first) +
                           "\t CRC checked: No error");
                fifo_out.push(out_block);
            } else { // checksum are different, drop packet if so
                logger.log("CRC checked: Error, RX Checksum: " + std::to_string(rx_crc) +
                           "\t Calculated Checksum: " + std::to_string(crc32.checksum()));
            }
        }
    }
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * CRC encode payload
 **********************************************************************/
void ecc_encode (tsFIFO<Block<bool>>& fifo_in,
                 tsFIFO<Block<bool>>& fifo_out,
                 size_t payload_size)
{
    // create logger
    Logger logger("EccEncode", "./ecc_encode.log");
    logger.logf("Payload size: " + std::to_string(payload_size));
    // Instantiate a crc-32 object
    boost::crc_32_type crc32;
    // create dummy block
    Block<bool> in_block;
    Block<bool> out_block;
    out_block.second.resize(payload_size+16+32+post_payload_len);
    logger.logf("Out block size: " + std::to_string(out_block.second.size()));
    std::vector<unsigned char> temp_char_vec;
    std::vector<bool> temp_bool_vec;
    temp_bool_vec.resize(16+payload_size);
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // pop input block
            fifo_in.pop(in_block);
            out_block.first = in_block.first;
            // create 16-bit packet number bitset
            std::bitset<16> packet_num_b(out_block.first);
            // push packet counter as 16-bit number
            for (size_t i=0; i<16; i++) {
                out_block.second[i] = packet_num_b[i];
                temp_bool_vec[i] = packet_num_b[i];
            }
            // getting payload from payload.h
            for (size_t i=0; i<payload_size; i++) {
                out_block.second[16+i] = in_block.second[i];
                temp_bool_vec[16+i] = in_block.second[i];
            }
            // convert vector of boolean to vector of unsigned char
            temp_char_vec = to_uchar_vec(temp_bool_vec);
            // CRC encode
            crc32.reset();
            crc32.process_bytes(temp_char_vec.data(), temp_char_vec.size());
            // get checksum and append to the end of payload
            boost::uint32_t crc_val = crc32.checksum();
            std::bitset<32> crc_bitset(crc32.checksum());
            logger.logf("Block: " + std::to_string(out_block.first) +
                       "\t TX Checksum: " + std::to_string(crc_val));
            // concatenate crc parity bits
            for (size_t i=0; i<32; i++)
                out_block.second[16+payload_size+i] = crc_bitset[i];
            // concatenate 28 zeros after ecc
            for (size_t i=0; i<post_payload_len; i++)
                out_block.second[16+payload_size+32+i] = 0;
            // push output block to fifo out
            fifo_out.push(out_block);
        }
    }
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * demodulation
 **********************************************************************/
void demod (tsFIFO<Block<std::complex<float>>>& fifo_in,
            tsFIFO<Block<bool>>& fifo_out,
            tsFIFO<std::pair<int, float>>& per_out,
            size_t demod_len,
            size_t payload_size)
{
    // create logger
    Logger logger("DEMOD", "./demod.log");
    logger.log("Demod length: " + std::to_string(demod_len));
    // create file to log demodulated packet
    std::ofstream out_file ("demod_out.dat", std::ofstream::binary);
    // create dummy block
    Block<std::complex<float>> in_block;
    Block<bool> out_block;
    out_block.first = 0;
    //out_block.second.resize(payload_size);
    out_block.second.resize(demod_len);
    bool demod_bit;
    float angle_cur = 0;
    float angle_pre = 0;
    float m_hat_0 = 0;
    float m_hat_1 = 0;
    //std::bitset<16> header;
    int ham_dist = 0;
    float ber = 0;
    std::pair<int, float> temp_per;
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // print out fifo size to check
            if (fifo_in.size() != 1)
                logger.logf("Demodulator input FIFO size: " + std::to_string(fifo_in.size()));
            if (fifo_out.size() != 0)
                logger.logf("Demodulator output FIFO size: " + std::to_string(fifo_out.size()));
            // pop input block from fifo
            fifo_in.pop(in_block);
            // non-coherent demodulation 
            for (size_t i=0; i<demod_len; i++) {
                angle_cur = std::arg(in_block.second[i+1]);
                angle_pre = std::arg(in_block.second[i]);
                m_hat_0 = std::cos(angle_cur-angle_pre);
                m_hat_1 = std::cos(angle_cur-angle_pre-M_PI);
                if (m_hat_0 > m_hat_1)
                    demod_bit = 0;
                else
                    demod_bit = 1;
                out_block.second[i] = demod_bit;
            }
            fifo_out.push(out_block);
        }
    }
    // close output file
    out_file.close();
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * acquisition
 **********************************************************************/
void acq (tsFIFO<Block<std::complex<float>>>& fifo_in,
          tsFIFO<Block<std::complex<float>>>& fifo_out,
          size_t input_block_len, size_t post_cap_len,
          size_t sym_per, size_t acq_len,
          float thresh)
{
    // create logger
    Logger logger("ACQ", "./acq.log");
    logger.logf("Symbol period: " + std::to_string(sym_per));
    // create a signature sequence vector
    std::vector<std::complex<float>> sig_seq_vec;
    sig_seq_vec.resize(sig_seq_len);
    for (size_t i=0; i<sig_seq_len; i++)
        sig_seq_vec[i] = sig_seq[i];
    // create dummy block
    Block<std::complex<float>> in_block;
    Block<std::complex<float>> out_block;
    out_block.second.resize(acq_len+1);
    // initialize parameters and temporary vectors
    size_t tao_end = input_block_len - acq_len * sym_per - post_cap_len;
    size_t tao_star = 0;
    size_t packet_start = 0;
    logger.logf("Tao end at: " + std::to_string(tao_end));
    std::pair<int, float> temp_pair;
    std::vector<std::complex<float>> temp;
    temp.resize(sig_seq_len);
    std::vector<float> corr_vec;
    corr_vec.resize(tao_end);
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // print out fifo size to check
            if (fifo_in.size() != 1)
                logger.logf("ACQ input FIFO size: " + std::to_string(fifo_in.size()));
            if (fifo_out.size() != 0)
                logger.logf("ACQ output FIFO size: " + std::to_string(fifo_out.size()));
            // pop input block from fifo
            fifo_in.pop(in_block);
            out_block.first = in_block.first;
            // fine symbol synchronization, i.e. find tao_star
            for (size_t t=0; t<tao_end; t++) {
                // get temp sequence r[n] = r(nT + tao)
                for (size_t i=0; i<sig_seq_len; i++)
                    temp[i] = in_block.second[i*sym_per+t];
                // compute correlation
                corr_vec[t] = correlation(sig_seq_vec, temp);
            }
            temp_pair = where_max(corr_vec);
            if (temp_pair.second > thresh) {
                tao_star = temp_pair.first;
                logger.logf("Tao_star: " + std::to_string(tao_star));
                // get decision statistic for demod
                packet_start = tao_star + 30*sym_per;
                logger.logf("Packet start: " + std::to_string(packet_start));
                for (size_t i=0; i<acq_len+1; i++)
                    out_block.second[i] = in_block.second[packet_start+i*sym_per];

                // push demod output to fifo
                fifo_out.push(out_block);
            }
        }
    }
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * Automatic gain control by normalizing RMS value
 **********************************************************************/
void per_count (tsFIFO<std::pair<int, float>>& fifo_in)
{
    // create logger
    Logger logger("PERCount", "./per_count.log");
    // create dummy block
    std::pair<int, float> in_block;
    size_t current_fifo_size = 0;
    size_t running_block_count = 0;
    size_t running_error_packet_count = 0;
    float per=0;
    while (not stop_signal_called) {
        // checking fifo size
        current_fifo_size = fifo_in.size();
        logger.log("In Block size: " + std::to_string(current_fifo_size));
        running_block_count += current_fifo_size;
        for (size_t i=0; i<current_fifo_size; i++) {
            fifo_in.pop(in_block);
            if (in_block.second > 0.0)
                running_error_packet_count++;
        }
        if (running_block_count != 0) {
            per = (float)running_error_packet_count/running_block_count;
            logger.log("Running packet count: " + std::to_string(running_block_count) +
                       "  Running PER: " + std::to_string(per));
        }
        // clear the fifo
        fifo_in.clear();
        // wait 10 second
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * Automatic gain control by normalizing RMS value
 **********************************************************************/
void agc (tsFIFO<Block<std::complex<float>>>& fifo_in,
          tsFIFO<Block<std::complex<float>>>& fifo_out,
          size_t block_size)
{
    // create logger
    Logger logger("AGC", "./agc.log");
    // create output filestream
    std::ofstream agc_in_file ("agc_in.dat", std::ofstream::binary);
    std::ofstream agc_out_file ("agc_out.dat", std::ofstream::binary);
    // create dummy block
    Block<std::complex<float>> in_block;
    Block<std::complex<float>> out_block;
    out_block.second.resize(block_size);
    float rms = 0;
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // print out fifo size to check
            if (fifo_in.size() != 1)
                logger.logf("AGC input FIFO size: " + std::to_string(fifo_in.size()));
            if (fifo_out.size() != 0)
                logger.logf("AGC output FIFO size: " + std::to_string(fifo_out.size()));
            // pop input block from fifo
            fifo_in.pop(in_block);
            // set block counter
            out_block.first = in_block.first;
            // calculate RMS value of block
            rms = 0;
            for (int i=0; i<block_size; i++) {
                rms += std::norm(in_block.second[i]);
            }
            rms /= block_size;
            rms = std::sqrt(rms);
            //logger.logf("Block: " + std::to_string(in_block.first) +
            //           " RMS: " + std::to_string(rms));
            // normalize by dividing each sample by rms value
            for (int i=0; i<block_size; i++) {
                out_block.second[i] = in_block.second[i]/rms;
            }
            // push block to fifo
            fifo_out.push(out_block);
            // store filter output to file to check with jupyter
            //agc_in_file.write((const char*)& in_block.second[0], block_size*sizeof(std::complex<float>));
            agc_out_file.write((const char*)& out_block.second[0], block_size*sizeof(std::complex<float>));
        }
    }
    // close output file
    agc_out_file.close();
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * Generate random bits as packets
 **********************************************************************/
void payload_gen (tsFIFO<Block<bool>>& fifo,
                 float p, size_t payload_size,
                 size_t packet_rate)
{
    // create logger
    Logger logger("PacketGen", "./payload_gen.log");
    int idle_time_us = (int)(1/(float)packet_rate * 1e6);
    logger.logf("Idle time: " + std::to_string(idle_time_us));
    // create block counter
    int block_counter = 0;
    // create dummy block
    Block<bool> block;
    block.second.resize(payload_size+16);
    while (not stop_signal_called) {
        // create 16-bit packet number bitset
        std::bitset<16> packet_num_b(block_counter);
        // set block counter
        block.first = block_counter++;
        // push packet counter as 16-bit number
        for (size_t i=0; i<16; i++) {
            block.second[i] = packet_num_b[i];
        }
        // getting payload from payload.h
        for (size_t i=0; i<payload_size; i++)
            block.second[16+i] = payload[i];
        //logger.log("Payload and header size: " + std::to_string(block.second.size()));
        // push block to fifo
        fifo.push(block);
        // print out fifo size to check
        if (fifo.size() != 1)
            logger.logf("Packet generator output FIFO size: " + std::to_string(fifo.size()));
        // wait
        std::this_thread::sleep_for(std::chrono::microseconds(idle_time_us));
    }
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * Modulation (BDPSK)
 **********************************************************************/
void modulate (tsFIFO<Block<bool>>& fifo_in,
               tsFIFO<Block<std::complex<float>>>& fifo_out,
               size_t in_block_size, size_t out_block_size)
{
    // create logger
    Logger logger("Modulator", "./mod.log");
    // create filestream
    //std::ofstream in_file ("mod_in.dat", std::ofstream::binary);
    std::ofstream out_file ("mod_out.dat", std::ofstream::binary);
    logger.log("Modulator input block size: " + std::to_string(in_block_size));
    logger.log("Modulator output block size: " + std::to_string(out_block_size));
    // create dummy block
    Block<bool> in_block;
    Block<std::complex<float>> out_block;
    out_block.second.resize(out_block_size);
    // prepend preamble
    for (int i=0; i<preamble_len; i++)
        out_block.second[i] = preamble[i];
    logger.logf("Preamble size: " + std::to_string(preamble_len));
    // prepend signature sequence
    for (int i=0; i<sig_seq_len; i++)
        out_block.second[i+preamble_len] = sig_seq[i];
    logger.logf("Signature sequence size: " + std::to_string(sig_seq_len));
    // create variables
    std::vector<std::complex<float>> temp_vec;
    temp_vec.resize(in_block_size+1);
    temp_vec[0] = sig_seq[sig_seq_len-1];
    float bpsk_sample;
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // print out fifo size to check
            if (fifo_in.size() != 1)
                logger.logf("Modulator input FIFO size: " + std::to_string(fifo_in.size()));
            if (fifo_out.size() != 0)
                logger.logf("Modulator output FIFO size: " + std::to_string(fifo_out.size()));
            // pop bit sequence from input fifo
            fifo_in.pop(in_block);
            out_block.first = in_block.first;
            // BDPSK modulation
            for (int i=0; i<in_block_size; i++) {
                bpsk_sample = -2.0 * (float)in_block.second[i] + 1.0;
                temp_vec[i+1] = bpsk_sample*temp_vec[i];
                out_block.second[i+preamble_len+sig_seq_len] = temp_vec[i+1];
                // log for debug
                //logger.log("Block: " + std::to_string(out_block.first) +
                //           " Bit: " + std::to_string(in_block.second[i]) +
                //           " Symbol: " + std::to_string(out_block.second[36+i].real()));
            }
            // push block to fifo
            fifo_out.push(out_block);
            // record samples to file
            //out_file.write((const char*)& out_block.second[0], block_size*sizeof(std::complex<float>));
        }
    }
    // close ofstream
    logger.log("Closing ofstream");
    //in_file.close();
    out_file.close();
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * Energy detector thread function
 **********************************************************************/
void energy_detector (tsFIFO<std::pair<Block<std::complex<float>>, Block<float>>>& fifo_in,
                      tsFIFO<Block<std::complex<float>>>& fifo_out,
                      size_t block_size, float threshold,
                      int cap_len, int pre_cap_len)
{
    // create logger
    Logger logger("EnergyDetector", "./energy_detector.log");
    // set up threshold checker param
    int total_len = cap_len+pre_cap_len;
    bool skip_f = false;
    int skip_len = cap_len;
    FixedQueue<std::complex<float>> pre_cap (pre_cap_len);
    int edge_mid_ind = 0;
    bool edge_f = false;
    int cap_block_num = 0;
    // create dummy blocks
    std::pair<Block<std::complex<float>>, Block<float>> in_pair;
    Block<std::complex<float>> in_block;   // block of raw samples
    Block<float> out_block;                // block of iir output samples
    out_block.second.resize(block_size);
    Block<std::complex<float>> cap_block;  // block of captured samples
    cap_block.second.resize(total_len);
    
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // check fifo sizes
            if (fifo_in.size() != 1)
                logger.logf("Energy detector input FIFO size: " + std::to_string(fifo_in.size()));
            if (fifo_out.size() != 0)
                logger.logf("Energy detector output FIFO size: " + std::to_string(fifo_out.size()));
            // pop fifo in
            fifo_in.pop(in_pair);
            in_block = in_pair.first;
            out_block = in_pair.second;
            // threshold_checker
            if (edge_f) {
                    for (int j=0; j<cap_len-edge_mid_ind; j++)  // capture the remaining samples
                        cap_block.second[pre_cap_len+edge_mid_ind+j] = in_block.second[j];
                    fifo_out.push(cap_block);   // push captured block to fifo out
                    edge_f = false;     // reset boundary case flag
                    skip_len = cap_len-edge_mid_ind;
            }
            for (int i=0; i<block_size; i++) {
                if (skip_f == false) {
                    if (out_block.second[i] > threshold) {  // check threshold
                        skip_f = true;
                        if (i > block_size-cap_len) {       // if capture range cross boundary
                            edge_f = true;                  // set boundary flag
                            edge_mid_ind = block_size-i;    // find mid point of capture range
                            for (int j=0; j<pre_cap_len; j++) // capture samples before packet
                                cap_block.second[j] = pre_cap.q[j];
                            for (int j=0; j<edge_mid_ind; j++)  // take the first few samples
                                cap_block.second[pre_cap_len+j] = in_block.second[i+j];
                            cap_block.first = cap_block_num++;  // assign, increment capture counter
                            //logger.log("Detected a packet at edge");
                            break;
                        } else {    // if capture range in middle
                            for (int j=0; j<pre_cap_len; j++) // capture samples before packet
                                cap_block.second[j] = pre_cap.q[j];
                            for (int j=0; j<cap_len; j++)   // capture 1k samples
                                cap_block.second[pre_cap_len+j] = in_block.second[i+j];
                            cap_block.first = cap_block_num++; // assign, increment capture counter
                            fifo_out.push(cap_block);   // push captured block to fifo out
                            //logger.log("Detected a packet in middle");
                        }
                    }
                } else {
                    skip_len = skip_len - 1;
                    if (skip_len == 0) {
                        skip_f = false;         // not skipping anymore
                        skip_len = cap_len;     // reset skip length
                    }
                }
                // keep pre-capture queue updated
                pre_cap.push(out_block.second[i]);
            }
        }
    }
    // notify user that processing thread is done
    logger.log("Closing");
}



/***********************************************************************
 * IIR filter thread function
 **********************************************************************/
void iir_filter (tsFIFO<Block<std::complex<float>>>& fifo_in,
                 tsFIFO<std::pair<Block<std::complex<float>>, Block<float>>>& fifo_out,
                 size_t block_size, float alpha)
{
    // create logger
    Logger logger("IIRFilter", "./iir_filter.log");
    // create output filestream
    std::ofstream iir_in_file ("iir_in.dat", std::ofstream::binary);
    std::ofstream iir_out_file ("iir_out.dat", std::ofstream::binary);
    // create dummy block
    Block<std::complex<float>> in_block;
    // IIR filter param
    float current_out = 0;
    float previous_out = 0;
    float current_out_db = 0;
    std::complex<float> sample;
    std::pair<Block<std::complex<float>>, Block<float>> out_pair;
    Block<float> iir_out_block;
    iir_out_block.second.resize(block_size);
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // check fifo sizes
            if (fifo_in.size() != 1)
                logger.logf("IIR filter input FIFO size: " + std::to_string(fifo_in.size()));
            if (fifo_out.size() != 0)
                logger.logf("IIR filter output FIFO size: " + std::to_string(fifo_out.size()));
            // iir
            // pop block from fifo
            fifo_in.pop(in_block);
            iir_out_block.first = in_block.first;
            // iir algorithm
            for (int i=0; i<block_size; i++) {
                sample = in_block.second[i];
                current_out = alpha*previous_out +
                       (1-alpha)*std::norm(sample);
                previous_out = current_out;
                iir_out_block.second[i] = current_out;
            }
            out_pair.first = in_block;
            out_pair.second = iir_out_block;
            fifo_out.push(out_pair);
            // put value in file
            iir_in_file.write((const char*)& in_block.second[0], block_size*sizeof(std::complex<float>));
            iir_out_file.write((const char*)& iir_out_block.second[0], block_size*sizeof(float));
        }
    }
    // close ofstream
    logger.log("Closing ofstream");
    iir_in_file.close();
    iir_out_file.close();
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * Filtering thread function
 **********************************************************************/
void filter(int D, int U, size_t in_len,
            std::vector<std::complex<float>>& filter_taps,
            size_t num_filt_threads, bool continuous,
            tsFIFO<Block<std::complex<float>>>& fifo_in,
            tsFIFO<Block<std::complex<float>>>& fifo_out)
{
    // create logger
    Logger logger("Filter", "./filter.log");
    // create output filestream
    std::ofstream in_file ("in_raw.dat", std::ofstream::binary);
    std::ofstream out_file ("out_raw.dat", std::ofstream::binary); 
    // print out down sampling and up sampling factors for debug
    logger.log("Down-sampling factor D: " + std::to_string(D));
    logger.log("Up-sampling factor U: " + std::to_string(U));
    // set up filter impulse response
    int h_len = filter_taps.size();
    std::complex<float>* h = &filter_taps[0];
    logger.log("Filter length: " + std::to_string(h_len));
    // print out taps for debug
    //for (int i=0; i<h_len; i++) {
    //    logger.logf("Tap: " + std::to_string(h[i].real()));
    //} 
    // test filter
    FilterPolyphase filt (U, D, in_len, h_len, h, num_filt_threads);
    int out_len = filt.out_len();
    logger.log("Filter output length: " + std::to_string(out_len));
    std::complex<float>* out = new std::complex<float>[out_len]();
    std::complex<float>* in = new std::complex<float>[in_len](); 
    // create dummy block and average variables
    Block<std::complex<float>> in_block;
    Block<std::complex<float>> out_block;
    // check ctrl-c and fifo empty
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // check fifo sizes
            if (fifo_in.size() != 1)
                logger.logf("Multirate filter input FIFO size: " + std::to_string(fifo_in.size()));
            if (fifo_out.size() != 0)
                logger.logf("Multirate filter output FIFO size: " + std::to_string(fifo_out.size()));
            // pop block from fifo
            fifo_in.pop(in_block);
            // get input array
            for (int i=0; i<in_len; i++) {
                in[i] = in_block.second[i];
            }
            // put value in file
            //in_file.write((const char*) in, in_len*sizeof(std::complex<float>));
            // filter
            if (continuous) {
                filt.set_head(in_block.first == 0);
                //logger.log("Continuous filtering: " + std::to_string(in_block.first==0));
            } else {
                filt.set_head(true);
                //logger.log("Burst filtering");
            }
            //filt.set_head(in_block.first == 0);
            filt.filter(in, out);
            // push filter output to fifo out
            out_block.first = in_block.first;
            out_block.second = std::vector<std::complex<float>>(out, out + out_len);
            fifo_out.push(out_block);
            // store filter output to file to check with jupyter
            //out_file.write((const char*) out, out_len*sizeof(std::complex<float>));
        }
    }
    // close ofstream
    logger.log("Closing ofstream");
    in_file.close();
    out_file.close();
    // notify user that processing thread is done
    logger.log("Closing");
}


/***********************************************************************
 * transmit_worker function
 * A function to be used as a boost::thread_group thread for transmitting
 **********************************************************************/
void transmit_worker (//size_t samp_per_buff,
                      size_t block_size,
                      uhd::tx_streamer::sptr tx_streamer,
                      //uhd::tx_metadata_t metadata,
                      tsFIFO<Block<std::complex<float>>>& fifo_in)
{
    // create a logger
    Logger logger("Tx Worker", "./tx-worker.log");
    // create zero vector to flush out buffer and create idle time at high packet rate
    std::vector<std::complex<float>> zeros;
    zeros.resize(300);
    std::fill(zeros.begin(), zeros.end(), 0);
    // create dummy block
    Block<std::complex<float>> block;
    // set up metadata
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = true;
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // print out tx fifo size
            if (fifo_in.size() != 1)
                logger.logf("TX fifo size: " + std::to_string(fifo_in.size()));
            // pop packet block
            fifo_in.pop(block);
            logger.log("Sending block: " + std::to_string(block.first));
            tx_streamer->send(&block.second.front(), block_size, md);
            // flush buffer
            //tx_streamer->send(&zeros.front(), 300, md);
        }
    }
    // send a mini EOB packet
    logger.log("Closing");
}


/***********************************************************************
 * recv_to_fifo function
 **********************************************************************/
void recv_to_fifo(uhd::usrp::multi_usrp::sptr usrp,
                  const std::string& cpu_format,
                  const std::string& wire_format,
                  const std::string& file,
                  size_t samps_per_buff,
                  int num_requested_samples,
                  double settling_time,
                  std::vector<size_t> rx_channel_nums,
                  tsFIFO<Block<std::complex<float>>>& fifo_in)
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
    std::vector<std::vector<std::complex<float>>> buffs(
        rx_channel_nums.size(), std::vector<std::complex<float>>(samps_per_buff));
    // create a vector of pointers to point to each of the channel buffers
    std::vector<std::complex<float>*> buff_ptrs;
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
    Block<std::complex<float>> block;
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
                           % (usrp->get_rx_rate() * sizeof(std::complex<float>) / 1e6);
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
                //(const char*)buff_ptrs[i], num_rx_samps * sizeof(std::complex<float>));
            //outfiles[i]->write((const char*)buff_ptrs[i], 3*sizeof(std::complex<float>));
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
    
    // tx variables
    std::string tx_args, tx_ant, tx_subdev, tx_channels, payload_filename;
    double tx_rate, ff_freq, tx_gain, tx_bw, tx_freq;
    int tx_D, tx_U;
    size_t rrc_half_len, packets_per_sec;
    size_t tx_packet_num_len;
    
    // rx variables to be set by po
    std::string ref, otw;
    std::string rx_args, file, rx_ant, rx_subdev, rx_channels;
    size_t total_num_samps, rx_spb;
    double rx_rate, fb_freq, rx_gain, rx_bw, rx_freq;
    double settling;
    int rx_D, rx_mf_U, ff_rx_cap_len, fb_rx_cap_len, rx_pre_cap_len, rx_post_cap_len;
    float alpha, ff_iir_threshold, ff_acq_threshold, fb_iir_threshold, fb_acq_threshold;
    //std::string taps_filename;

    // other variables
    bool tx_rx = false;
    
    // program specific variables
    size_t num_pa_threads, num_filt_threads;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("tx-rx", po::value<bool>(&tx_rx), "flag to indicate if radio is tx (1) or rx (0)")
        ("tx-args", po::value<std::string>(&tx_args)->default_value(""), "uhd transmit device address args")
        ("rx-args", po::value<std::string>(&rx_args)->default_value(""), "uhd receive device address args")
        ("file", po::value<std::string>(&file)->default_value("usrp_samples.dat"), "name of the file to write binary samples to")
        //("type", po::value<std::string>(&type)->default_value("float"), "sample type in file: double, float, or short")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("settling", po::value<double>(&settling)->default_value(double(0.2)), "settling time (seconds) before receiving")
        ("tx-rate", po::value<double>(&tx_rate)->default_value(1000000), "rate of transmit outgoing samples")
        ("ff-freq", po::value<double>(&ff_freq)->default_value(915000000), "Feedforward channel center frequency in Hz")
        ("tx-gain", po::value<double>(&tx_gain)->default_value(20), "gain for the transmit RF chain")
        ("rx-spb", po::value<size_t>(&rx_spb)->default_value(10000), "samples per buffer")
        ("rx-rate", po::value<double>(&rx_rate)->default_value(1000000), "rate of receive incoming samples")
        ("fb-freq", po::value<double>(&fb_freq)->default_value(2412000000), "Feedback channel center frequency in Hz")
        ("rx-gain", po::value<double>(&rx_gain)->default_value(20), "gain for the receive RF chain")
        ("tx-ant", po::value<std::string>(&tx_ant), "transmit antenna selection")
        ("rx-ant", po::value<std::string>(&rx_ant), "receive antenna selection")
        ("tx-subdev", po::value<std::string>(&tx_subdev), "transmit subdevice specification")
        ("rx-subdev", po::value<std::string>(&rx_subdev), "receive subdevice specification")
        ("tx-bw", po::value<double>(&tx_bw), "analog transmit filter bandwidth in Hz")
        ("rx-bw", po::value<double>(&rx_bw), "analog receive filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external, mimo)")
        ("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode")
        ("tx-channels", po::value<std::string>(&tx_channels)->default_value("0"), "which TX channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("rx-channels", po::value<std::string>(&rx_channels)->default_value("0"), "which RX channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("tx-int-n", "tune USRP TX with integer-N tuning")
        ("rx-int-n", "tune USRP RX with integer-N tuning")
        ("tx-D,tx-d", po::value<int>(&tx_D)->default_value(4), "Tx side down-sampling factor")
        ("tx-U,tx-u", po::value<int>(&tx_U)->default_value(5), "Tx side up-sampling factor")
        ("rx-D,rx-d", po::value<int>(&rx_D)->default_value(1), "Rx side down-sampling factor")
        ("rx-mf-U", po::value<int>(&rx_mf_U)->default_value(4), "Rx side match filter up-sampling factor")
        ("alpha", po::value<float>(&alpha)->default_value(0.3), "IIR smoothing coefficient")
        ("ff-iir-thresh,ff-iir-threshold", po::value<float>(&ff_iir_threshold)->default_value(0.001), "Threshold for energy detector")
        ("ff-acq-thresh,ff-acq-threshold", po::value<float>(&ff_acq_threshold)->default_value(15), "Threshold for correlation in acquisition")
        ("fb-iir-thresh,fb-iir-threshold", po::value<float>(&fb_iir_threshold)->default_value(0.01), "Threshold for energy detector")
        ("fb-acq-thresh,fb-acq-threshold", po::value<float>(&fb_acq_threshold)->default_value(15), "Threshold for correlation in acquisition")
        //("taps-file", po::value<std::string>(&taps_filename), "filepath of filter taps file")
        ("n-filt-threads", po::value<size_t>(&num_filt_threads)->default_value(1), "number of threads for filtering")
        ("n-pa-threads", po::value<size_t>(&num_pa_threads)->default_value(1), "number of threads for power averaging")
        ("rrc-half-len", po::value<size_t>(&rrc_half_len)->default_value(50), "Tx side down-sampling factor")
        //("tx-packet-num-len", po::value<size_t>(&tx_packet_num_len)->default_value(16), "Tx side length of packet number in bits")
        ("ff-rx-cap-len", po::value<int>(&ff_rx_cap_len)->default_value((36+16+payload_len+32)*5/4), "Rx capture length without front extension for DATA")
        ("fb-rx-cap-len", po::value<int>(&fb_rx_cap_len)->default_value((36+16+0+32)*5/4), "Rx capture length without front extension for ACK")
        ("rx-pre-cap-len", po::value<int>(&rx_pre_cap_len)->default_value(20), "Front extension length of rx capture")
        ("rx-post-cap-len", po::value<int>(&rx_post_cap_len)->default_value(200), "Back extension length of rx capture")
        ("packets-per-sec", po::value<size_t>(&packets_per_sec)->default_value(1), "Transmit packets per seconds (max 800)")
        ("payload", po::value<std::string>(&payload_filename)->default_value("payload.jpeg"), "File to transmit")
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

    // check if user specify tx rx
    if (not vm.count("tx-rx")) {
        std::cerr << "Please specify transmit or receive side with --tx-rx"
                  << std::endl;
        return ~0;
    }
    
    // check packets per sec
    if (packets_per_sec > 800) {
        std::cerr << "Invalid packets per second, please retry with a different value"
                  << std::endl;
        return ~0;
    }
    
    // check payload filename
    if (not vm.count("payload")) {
        std::cerr << "Please specify the file to transmit with --payload"
                  << std::endl;
        return ~0;
    }
    
    // create a usrp device
    main_logger.log("Creating the transmit usrp device with: " + tx_args);
    uhd::usrp::multi_usrp::sptr tx_usrp = uhd::usrp::multi_usrp::make(tx_args);
    main_logger.log("Creating the receive usrp device with: " + rx_args);
    uhd::usrp::multi_usrp::sptr rx_usrp = uhd::usrp::multi_usrp::make(rx_args);

    // always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("tx-subdev"))
        tx_usrp->set_tx_subdev_spec(tx_subdev);
    if (vm.count("rx-subdev"))
        rx_usrp->set_rx_subdev_spec(rx_subdev);

    // detect which channels to use
    std::vector<std::string> tx_channel_strings;
    std::vector<size_t> tx_channel_nums;
    boost::split(tx_channel_strings, tx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < tx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(tx_channel_strings[ch]);
        if (chan >= tx_usrp->get_tx_num_channels()) {
            throw std::runtime_error("Invalid TX channel(s) specified.");
        } else
            tx_channel_nums.push_back(std::stoi(tx_channel_strings[ch]));
    }
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
        tx_usrp->set_clock_source(ref);
        rx_usrp->set_clock_source(ref);
    }
    main_logger.log("Using TX Device: " + tx_usrp->get_pp_string());
    main_logger.log("Using RX Device: " + rx_usrp->get_pp_string());
    
    // set the transmit sample rate
    if (not vm.count("tx-rate")) {
        std::cerr << "Please specify the transmit sample rate with --tx-rate"
                  << std::endl;
        return ~0;
    }
    main_logger.log("Setting TX Rate: " + std::to_string(tx_rate / 1e6) + "Msps");
    tx_usrp->set_tx_rate(tx_rate);
    main_logger.log("Actual TX Rate: " + std::to_string(tx_usrp->get_tx_rate() / 1e6) + "Msps");
    
    // set the receive sample rate
    if (not vm.count("rx-rate")) {
        std::cerr << "Please specify the sample rate with --rx-rate" << std::endl;
        return ~0;
    }
    main_logger.log("Setting RX Rate: " + std::to_string(rx_rate/1e6) + "Msps");
    rx_usrp->set_rx_rate(rx_rate);
    main_logger.log("Actual RX Rate: " + std::to_string(rx_usrp->get_rx_rate()/1e6) + "Msps");

    // set the feedforward center frequency
    if (not vm.count("ff-freq")) {
        std::cerr << "Please specify the feedforward center frequency with --ff-freq"
                  << std::endl;
        return ~0;
    }
    
    // set the feedback center frequency
    if (not vm.count("fb-freq")) {
        std::cerr << "Please specify the feedback center frequency with --fb-freq"
                  << std::endl;
        return ~0;
    }
    
    // check feedforward or feedback
    if (tx_rx == 1) { // feed forward
        tx_freq = ff_freq;
        rx_freq = fb_freq;
    } else {
        tx_freq = fb_freq;
        rx_freq = ff_freq;
    }
    
    // set transmit args
    for (size_t ch = 0; ch < tx_channel_nums.size(); ch++) {
        size_t channel = tx_channel_nums[ch];
        if (tx_channel_nums.size() > 1) {
            main_logger.log("Configuring TX Channel " + std::to_string(channel));
        }
        main_logger.log("Setting TX Freq: " + std::to_string(tx_freq / 1e6) + "MHz");
        // check feedforward or feedback
        uhd::tune_request_t tx_tune_request(tx_freq);
        if (vm.count("tx-int-n"))
            tx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        tx_usrp->set_tx_freq(tx_tune_request, channel);
        main_logger.log("Actual TX Freq: " +
                        std::to_string(tx_usrp->get_tx_freq(channel) / 1e6) + "MHz");
        // set the rf gain
        if (vm.count("tx-gain")) {
            main_logger.log("Setting TX Gain: " + std::to_string(tx_gain) + "dB");
            tx_usrp->set_tx_gain(tx_gain, channel);
            main_logger.log("Actual TX Gain: " +
                            std::to_string(tx_usrp->get_tx_gain(channel)) + "dB");
        }
        // set the analog frontend filter bandwidth
        if (vm.count("tx-bw")) {
            main_logger.log("Setting TX Bandwidth: " + std::to_string(tx_bw) + "MHz");
            tx_usrp->set_tx_bandwidth(tx_bw, channel);
            main_logger.log("Actual TX Bandwidth: " +
                            std::to_string(tx_usrp->get_tx_bandwidth(channel)) + "MHz");
        }
        // set the antenna
        if (vm.count("tx-ant"))
            tx_usrp->set_tx_antenna(tx_ant, channel);
    }
    
    // set receive args
    for (size_t ch = 0; ch < rx_channel_nums.size(); ch++) {
        size_t channel = rx_channel_nums[ch];
        if (rx_channel_nums.size() > 1) {
            main_logger.log("Configuring RX Channel " + std::to_string(channel));
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
    
    // create a transmit streamer
    // linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32", otw);
    stream_args.channels             = tx_channel_nums;
    uhd::tx_streamer::sptr tx_stream = tx_usrp->get_tx_stream(stream_args);

    // Check Ref and LO Lock detect
    std::vector<std::string> tx_sensor_names, rx_sensor_names;
    tx_sensor_names = tx_usrp->get_tx_sensor_names(0);
    if (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "lo_locked")
        != tx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = tx_usrp->get_tx_sensor("lo_locked", 0);
        //std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string()
        //          << std::endl;
        main_logger.log("Checking TX: " + lo_locked.to_pp_string());
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    rx_sensor_names = rx_usrp->get_rx_sensor_names(0);
    if (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "lo_locked")
        != rx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = rx_usrp->get_rx_sensor("lo_locked", 0);
        //std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string()
        //          << std::endl;
        main_logger.log("Checking RX: " + lo_locked.to_pp_string());
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }

    tx_sensor_names = tx_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "mimo_locked")
                != tx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = tx_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "ref_locked")
                != tx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = tx_usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    rx_sensor_names = rx_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "mimo_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = rx_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "ref_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = rx_usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }
    
    // reset usrp time to prepare for transmit/receive
    main_logger.log("Setting device timestamp to 0");
    tx_usrp->set_time_now(uhd::time_spec_t(0.0));
    //rx_usrp->set_time_now(uhd::time_spec_t(0.0));
    
    // set up tx threads worker
    std::thread payload_gen_t;
    std::thread src_arq_schedule_t;
    std::thread src_arq_schedule_test_t;
    std::thread ecc_encode_t;
    std::thread pulse_shaper_t;
    std::thread modulator_t;
    std::thread tx_worker_t;
    
    // set up rx threads worker
    std::thread mf_worker_t;
    std::thread iir_filter_worker_t;
    std::thread energy_detector_t;
    std::thread power_average_worker_t[num_pa_threads];
    std::thread agc_t;
    std::thread acq_t;
    std::thread demod_t;
    std::thread per_count_t;
    std::thread captured_block_count_t;
    std::thread ecc_decode_t;
    std::thread snk_arq_schedule_t;
    std::thread ack_prepare_t;
    std::thread rx_worker_t;
    
    // init packet len
    size_t ff_tx_packet_len = preamble_len + sig_seq_len + 16 + payload_len + 32 + post_payload_len;
    size_t fb_tx_packet_len = preamble_len + sig_seq_len + 16 + 32 + post_payload_len;
    main_logger.log("Total data packet size: " + std::to_string(ff_tx_packet_len));
    main_logger.log("Total ack packet size: " + std::to_string(fb_tx_packet_len));
    
    // make an array pointer to hold pulse shape filter
    size_t rrc_len = 2*rrc_half_len+1;
    std::complex<float> rrc_h[rrc_len];
    // call root raised cosine function
    rrc_pulse(rrc_h, rrc_half_len, tx_U, tx_D);
    std::vector<std::complex<float>> rrc_vec(rrc_h, rrc_h + rrc_len);
    // store rrc impulse in file
    std::ofstream rrc_file ("rrc.dat", std::ofstream::binary);
    rrc_file.write((const char*) rrc_h, rrc_len*sizeof(std::complex<float>));
    rrc_file.close();

    // recv to file as function
    if (!tx_rx) {      // RX SIDE        
        // fifo for feed forward stream
        tsFIFO<Block<std::complex<float>>> fifo_in;
        tsFIFO<std::pair<Block<std::complex<float>>, Block<float>>> iir_out_fifo;
        tsFIFO<Block<std::complex<float>>> energy_detector_out_fifo;
        tsFIFO<Block<std::complex<float>>> agc_out_fifo;
        tsFIFO<Block<std::complex<float>>> mf_out_fifo;
        tsFIFO<Block<std::complex<float>>> acq_out_fifo;
        tsFIFO<Block<bool>> demod_out_fifo;
        tsFIFO<Block<bool>> decode_out_fifo;
        tsFIFO<Block<bool>> data_fifo;
        tsFIFO<std::pair<int, float>> per_fifo;
        tsFIFO<Block<bool>> empty_fifo;
        // fifo for feedback stream
        tsFIFO<Block<bool>> ecc_fifo;
        tsFIFO<Block<bool>> ack_fifo;
        tsFIFO<Block<std::complex<float>>> mod_fifo;
        tsFIFO<Block<std::complex<float>>> pulse_shape_out_fifo;
        tsFIFO<Block<std::complex<float>>> arq_fifo;
        tsFIFO<Block<std::complex<float>>> arq_test_fifo;
        
        // FEEDBACK TX STREAM
        // spawn transmit worker thread
        tx_worker_t = std::thread(&transmit_worker, fb_tx_packet_len*tx_U/tx_D,
                std::ref(tx_stream), std::ref(arq_fifo));
        // instantiate pulse shaping filter as multirate filter
        pulse_shaper_t = std::thread(&filter, tx_D, tx_U, fb_tx_packet_len,
                std::ref(rrc_vec), num_filt_threads, false,
                std::ref(mod_fifo), std::ref(pulse_shape_out_fifo));
        // spawn modulation thread
        modulator_t = std::thread(&modulate, std::ref(ecc_fifo),
                std::ref(mod_fifo), 16+32+post_payload_len,
                fb_tx_packet_len);
        // spawn error control thread
        ecc_encode_t = std::thread(&ecc_encode, std::ref(ack_fifo),
                std::ref(ecc_fifo), 0);
        // call function to prep all ack packets, assume number of packets is known
        ack_prepare(ack_fifo, 1082);
        
        // FEED FORWARD RX STREAM
        // create thread for power averager
        //iir_filter_worker_t = std::thread(&iir_filter, std::ref(fifo_in),
        //        std::ref(iir_out_fifo), rx_spb, alpha);
        //// create thread for energy detector
        //energy_detector_t = std::thread(&energy_detector, std::ref(iir_out_fifo),
        //        std::ref(energy_detector_out_fifo), rx_spb,
        //        ff_iir_threshold, ff_rx_cap_len+rx_post_cap_len, rx_pre_cap_len);
        //// create thread for multirate filtering
        //mf_worker_t = std::thread(&filter, 1, rx_mf_U,
        //        ff_rx_cap_len+rx_pre_cap_len+rx_post_cap_len,
        //        std::ref(rrc_vec), num_filt_threads, false,
        //        std::ref(energy_detector_out_fifo), std::ref(mf_out_fifo));
        //// create thread for agc
        //agc_t = std::thread(&agc, std::ref(mf_out_fifo),
        //        std::ref(agc_out_fifo),
        //        (ff_rx_cap_len+rx_pre_cap_len+rx_post_cap_len)*rx_mf_U);
        //// create thread for acquistion
        //acq_t = std::thread(&acq, std::ref(agc_out_fifo), std::ref(acq_out_fifo),
        //        (ff_rx_cap_len+rx_pre_cap_len+rx_post_cap_len)*rx_mf_U,
        //        rx_post_cap_len, rx_mf_U*5/4, payload_len+16+32,
        //        ff_acq_threshold);
        //// create thread for demodulation
        //demod_t = std::thread(&demod, std::ref(acq_out_fifo), std::ref(demod_out_fifo),
        //        std::ref(per_fifo), payload_len+16+32, payload_len+32);
        //// create thread for ecc decode
        //ecc_decode_t = std::thread(&ecc_decode, std::ref(demod_out_fifo),
        //        std::ref(decode_out_fifo), payload_len+16);
        //// spawn thread for sink arq scheduler
        snk_arq_schedule_t = std::thread(&snk_arq_schedule, std::ref(decode_out_fifo),
                std::ref(pulse_shape_out_fifo), std::ref(data_fifo),
                std::ref(arq_fifo), payload_len);
        // call receive function
        recv_to_fifo(rx_usrp, "fc32", otw, file,
            rx_spb, total_num_samps, settling,
            rx_channel_nums, fifo_in);

    } else {      // TX SIDE
        // fifo for feedforward stream
        tsFIFO<Block<unsigned char>> char_payload_fifo;
        tsFIFO<Block<bool>> bit_payload_fifo;
        tsFIFO<Block<bool>> ack_fifo;
        tsFIFO<Block<bool>> ecc_fifo;
        tsFIFO<Block<std::complex<float>>> mod_fifo;
        tsFIFO<Block<std::complex<float>>> pulse_shape_out_fifo;
        tsFIFO<Block<std::complex<float>>> arq_fifo;
        tsFIFO<Block<std::complex<float>>> arq_test_fifo;
        // fifo for feed back stream
        tsFIFO<Block<std::complex<float>>> fifo_in;
        tsFIFO<std::pair<Block<std::complex<float>>, Block<float>>> iir_out_fifo;
        tsFIFO<Block<std::complex<float>>> energy_detector_out_fifo;
        tsFIFO<Block<std::complex<float>>> agc_out_fifo;
        tsFIFO<Block<std::complex<float>>> mf_out_fifo;
        tsFIFO<Block<std::complex<float>>> acq_out_fifo;
        tsFIFO<Block<bool>> demod_out_fifo;
        tsFIFO<Block<bool>> decode_out_fifo;
        tsFIFO<Block<bool>> data_fifo;
        tsFIFO<std::pair<int, float>> per_fifo;
        tsFIFO<Block<bool>> empty_fifo;
        
        // FEED BACK RX STREAM
        // create thread for power averager
        iir_filter_worker_t = std::thread(&iir_filter, std::ref(fifo_in),
                std::ref(iir_out_fifo), rx_spb, alpha);
        // create thread for energy detector
        energy_detector_t = std::thread(&energy_detector, std::ref(iir_out_fifo),
                std::ref(energy_detector_out_fifo), rx_spb,
                fb_iir_threshold, fb_rx_cap_len+rx_post_cap_len, rx_pre_cap_len);
        // create thread for multirate filtering
        mf_worker_t = std::thread(&filter, 1, rx_mf_U,
                fb_rx_cap_len+rx_pre_cap_len+rx_post_cap_len,
                std::ref(rrc_vec), num_filt_threads, false,
                std::ref(energy_detector_out_fifo), std::ref(mf_out_fifo));
        // create thread for agc
        agc_t = std::thread(&agc, std::ref(mf_out_fifo),
                std::ref(agc_out_fifo),
                (fb_rx_cap_len+rx_pre_cap_len+rx_post_cap_len)*rx_mf_U);
        // create thread for acquistion
        acq_t = std::thread(&acq, std::ref(agc_out_fifo), std::ref(acq_out_fifo),
                (fb_rx_cap_len+rx_pre_cap_len+rx_post_cap_len)*rx_mf_U,
                rx_post_cap_len, rx_mf_U*5/4, 0+16+32,
                fb_acq_threshold);
        // create thread for demodulation
        demod_t = std::thread(&demod, std::ref(acq_out_fifo), std::ref(demod_out_fifo),
                std::ref(per_fifo), 0+16+32, 0+32);
        // create thread for ecc decode
        ecc_decode_t = std::thread(&ecc_decode, std::ref(demod_out_fifo),
                std::ref(ack_fifo), 0+16);
        // call receive function
        rx_worker_t = std::thread(&recv_to_fifo, std::ref(rx_usrp),
            "fc32", std::ref(otw), std::ref(file),
            rx_spb, total_num_samps, settling,
            rx_channel_nums, std::ref(fifo_in));
        
        // FEED FORWARD TX STREAM
        // spawn thread for source arq scheduler
        //src_arq_schedule_t = std::thread(&src_arq_schedule,
        //        std::ref(pulse_shape_out_fifo), std::ref(arq_fifo),
        //        std::ref(ack_fifo));
        //// instantiate pulse shaping filter as multirate filter
        //pulse_shaper_t = std::thread(&filter, tx_D, tx_U, ff_tx_packet_len,
        //        std::ref(rrc_vec), num_filt_threads, false,
        //        std::ref(mod_fifo), std::ref(pulse_shape_out_fifo));
        //// spawn modulation thread
        //modulator_t = std::thread(&modulate, std::ref(ecc_fifo),
        //        std::ref(mod_fifo), 16+payload_len+32+post_payload_len,
        //        ff_tx_packet_len);
        //// spawn error control thread
        //ecc_encode_t = std::thread(&ecc_encode, std::ref(bit_payload_fifo),
        //        std::ref(ecc_fifo), payload_len);
        //// call function to read in payload file
        //read_payload(payload_filename, char_payload_fifo, bit_payload_fifo, 125);
            
        // call tx worker function as main thread
        transmit_worker(ff_tx_packet_len*tx_U/tx_D, tx_stream,
                        arq_fifo);
    }
    
    // clean up transmit worker
    stop_signal_called = true;
    if (mf_worker_t.joinable())
        mf_worker_t.join();
    if (iir_filter_worker_t.joinable())
        iir_filter_worker_t.join();
    if (agc_t.joinable())
        agc_t.join();
    if (captured_block_count_t.joinable())
        captured_block_count_t.join();
    for (int i=0; i<num_pa_threads; i++) {
        if (power_average_worker_t[i].joinable())
            power_average_worker_t[i].join();
    }
    if (payload_gen_t.joinable())
        payload_gen_t.join();
    if (modulator_t.joinable())
        modulator_t.join();
    if (pulse_shaper_t.joinable())
        pulse_shaper_t.join();
    
    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}