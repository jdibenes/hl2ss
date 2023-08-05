
#include <Windows.h>
#include <stdio.h>
#include <malloc.h>
#include "log.h"
#include <spdlog/spdlog.h>
#include "spdlog/sinks/base_sink.h"
#include "spdlog/details/log_msg_buffer.h"
#include "spdlog/details/null_mutex.h"
#include "spdlog/sinks/rotating_file_sink.h"


//-----------------------------------------------------------------------------
// Functions 
//-----------------------------------------------------------------------------

std::string wide_string_to_string(const std::wstring& wide_string)
{
	if (wide_string.empty())
	{
		return "";
	}

	const auto size_needed = WideCharToMultiByte(CP_UTF8, 0, &wide_string.at(0), (int)wide_string.size(), nullptr, 0, nullptr, nullptr);
	if (size_needed <= 0)
	{
		throw std::runtime_error("WideCharToMultiByte() failed: " + std::to_string(size_needed));
	}

	std::string result(size_needed, 0);
	WideCharToMultiByte(CP_UTF8, 0, &wide_string.at(0), (int)wide_string.size(), &result.at(0), size_needed, nullptr, nullptr);
	// maybe the conversion is a bit buggy .. seems that there is a \0 byte somewhere ..
	return std::string(result.c_str());
}


enum class LogColor { Red, Green, Blue, Black, White, Yellow, Orange };
namespace spdlog {
	namespace sinks {

		template<typename Mutex>
		class cb_logging_sink final : public base_sink<Mutex>
		{

		public:
			explicit cb_logging_sink(LoggingFuncCallBack cb)
			{
				log_cb = cb;
			}

		protected:
			void sink_it_(const details::log_msg& msg) override
			{
				try {
					memory_buf_t formatted;
					base_sink<Mutex>::formatter_->format(msg, formatted);
					std::string msg_str = fmt::to_string(formatted);

					LogColor color{ LogColor::Black };
					switch (msg.level) {
					case spdlog::level::level_enum::critical:
					case spdlog::level::level_enum::err:
					case spdlog::level::level_enum::warn:
						color = LogColor::Red;
						break;
					default:
						color = LogColor::Black;
					}

					log_cb(msg_str.c_str(), (int)color, static_cast<int>(msg_str.size()));
				}
				catch (const std::exception& e) {
					OutputDebugStringA(e.what());
				}
			}
			void flush_() override {
				// nothing to do here
			}

		private:
			LoggingFuncCallBack log_cb;
		};

		using cb_logging_sink_mt = cb_logging_sink<std::mutex>;
		using cb_logging_sink_st = cb_logging_sink<details::null_mutex>;



		template<typename Mutex>
		class console_logging_sink final : public base_sink<Mutex>
		{

		public:
			console_logging_sink() = default;

		protected:
			void sink_it_(const details::log_msg& msg) override
			{
				memory_buf_t formatted;
				base_sink<Mutex>::formatter_->format(msg, formatted);
				std::string msg_str = fmt::to_string(formatted);
				OutputDebugStringA(msg_str.c_str());
			}
			void flush_() override {
				// nothing to do here
			}

		private:
		};

		using console_logging_sink_mt = console_logging_sink<std::mutex>;
		using console_logging_sink_st = console_logging_sink<details::null_mutex>;





	} // namespace sinks
}

void SetupCallbackLogSink(LoggingFuncCallBack cb) {
	auto cbs = std::make_shared< spdlog::sinks::cb_logging_sink_mt>(cb);
	std::vector<spdlog::sink_ptr>& sinks = spdlog::details::registry::instance().get_default_raw()->sinks();
	sinks.push_back(cbs);
}

void SetupDebugLogSink() {
	auto cbs = std::make_shared< spdlog::sinks::console_logging_sink_mt>();
	cbs->set_pattern("%H:%M:%S.%e [%L] %v (%s:%#)");
	std::vector<spdlog::sink_ptr>& sinks = spdlog::details::registry::instance().get_default_raw()->sinks();
	sinks.push_back(cbs);
}

void SetupFileLogSink(const char* filename) {
	auto max_size = 1048576 * 100;
	auto max_files = 3;
	auto filesink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(filename, max_size, max_files);
	filesink->set_pattern("%H:%M:%S.%e [%L] %v (%s:%#)");
	std::vector<spdlog::sink_ptr>& sinks = spdlog::details::registry::instance().get_default_raw()->sinks();
	sinks.push_back(filesink);
}


//// OK
//void ShowMessage(const char* format, ...)
//{
//	spdlog::debug(format, )
//	char* text;
//	int len;
//	va_list arg_list;
//	va_start(arg_list, format);
//	len = _vscprintf(format, arg_list) + 2;
//	text = (char*)malloc(len);
//	if (!text) { return; }
//	vsprintf_s(text, len, format, arg_list);
//	va_end(arg_list);
//	text[len - 2] = '\n';
//	text[len - 1] = '\0';
//	OutputDebugStringA(text);
//	free(text);
//}
//
//// OK
//void ShowMessage(const wchar_t* format, ...)
//{
//	wchar_t* text;
//	int len;
//	va_list arg_list;
//	va_start(arg_list, format);
//	len = _vscwprintf(format, arg_list) + 2;
//	text = (wchar_t*)malloc(len * sizeof(wchar_t));
//	if (!text) { return; }
//	vswprintf_s(text, len, format, arg_list);
//	va_end(arg_list);
//	text[len - 2] = L'\n';
//	text[len - 1] = L'\0';
//	OutputDebugStringW(text);
//	free(text);
//}
