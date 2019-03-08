/**
 * @file Utils/include/ZLIBCompression.h
 *
 * This file declares the utility functions for compressing data
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <iostream>
#include <sstream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/numeric/conversion/cast.hpp>

#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <zlib.h>

namespace Utils {

  void compress(const std::string& data, std::string& buffer)
  {
      boost::iostreams::filtering_streambuf<boost::iostreams::output> out;
      out.push(boost::iostreams::zlib_compressor());
      out.push(boost::iostreams::back_inserter(buffer));
      boost::iostreams::copy(boost::make_iterator_range(data), out);
  }

  void decompress(const std::string& data, std::string& buffer)
  {
      boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
      in.push(boost::iostreams::zlib_decompressor());
      in.push(boost::make_iterator_range(data));
      boost::iostreams::copy(in, boost::iostreams::back_inserter(buffer));
  }

}
