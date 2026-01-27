#include "core/logging.hpp"
#include "core/log_names.hpp"

#include "logger/console_sink.hpp"
#include "logger/file_sink.hpp"
#include "logger/logger.hpp"
#include "logger/logger_registry.hpp"
#include "logger/pattern_formatter.hpp"

#include <memory>
#include <string>

namespace bullseye_pred::logging
{

void init(const Config& cfg)
{
    using namespace sim_logger;

    auto root = LoggerRegistry::instance().get_logger(bullseye_pred::logname::kRoot);
    root->set_level(cfg.level);

    PatternFormatter fmt(cfg.pattern);
    auto console = std::make_shared<ConsoleSink>(fmt, ConsoleSink::ColorMode::Auto);

    if (!cfg.file_path.empty())
    {
        auto file = std::make_shared<FileSink>(cfg.file_path, fmt, /*durable_flush=*/false);
        root->set_sinks({console, file});
    }
    else
    {
        root->set_sinks({console});
    }

    root->set_immediate_flush(cfg.immediate_flush);
}

std::shared_ptr<sim_logger::Logger> get(std::string_view component)
{
    using namespace sim_logger;

    std::string name = bullseye_pred::logname::kRoot;
    if (!component.empty())
    {
        name.push_back('.');
        name.append(component.data(), component.size());
    }
    return LoggerRegistry::instance().get_logger(name);
}

} // namespace bullseye_pred::logging
