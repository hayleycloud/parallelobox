#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <optional>

class Arguments
{
public:
	Arguments(int argc, const char** argv);

	[[nodiscard]] size_t count(bool inclPath = true) const;

	[[nodiscard]] std::optional<std::string> get(std::string_view arg) const;

	[[nodiscard]] bool getFlag(std::string_view flag) const;

	[[nodiscard]] std::optional<int> getInt(std::string_view arg) const;

	[[nodiscard]] std::optional<float> getFloat(std::string_view arg) const;

private:
	std::vector<std::string> m_ArgSet;
};

 