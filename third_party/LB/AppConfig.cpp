#include "AppConfig.h"

#include <Windows.h>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>

namespace
{
	typedef std::map<std::string, std::string> IniMap;

	cv::Mat MakeMat(int rows, int cols, const double* values)
	{
		cv::Mat mat(rows, cols, CV_64F);
		for (int r = 0; r < rows; ++r)
		{
			for (int c = 0; c < cols; ++c)
			{
				mat.at<double>(r, c) = values[r * cols + c];
			}
		}
		return mat;
	}

	std::string Trim(const std::string& text)
	{
		size_t first = 0;
		while (first < text.size() && std::isspace((unsigned char)text[first]))
		{
			++first;
		}

		size_t last = text.size();
		while (last > first && std::isspace((unsigned char)text[last - 1]))
		{
			--last;
		}

		return text.substr(first, last - first);
	}

	std::string ToLowerAscii(std::string text)
	{
		for (size_t i = 0; i < text.size(); ++i)
		{
			text[i] = (char)std::tolower((unsigned char)text[i]);
		}
		return text;
	}

	std::string StripInlineComment(const std::string& text)
	{
		bool quote = false;
		for (size_t i = 0; i < text.size(); ++i)
		{
			const char ch = text[i];
			if (ch == '"')
			{
				quote = !quote;
				continue;
			}
			if (!quote && (ch == ';' || ch == '#'))
			{
				if (i == 0 || std::isspace((unsigned char)text[i - 1]))
				{
					return Trim(text.substr(0, i));
				}
			}
		}
		return Trim(text);
	}

	void CommitPendingIniValue(IniMap& values, std::string& pending_key, std::string& pending_value)
	{
		if (!pending_key.empty())
		{
			values[pending_key] = Trim(pending_value);
			pending_key.clear();
			pending_value.clear();
		}
	}

	std::wstring Utf8ToWide(const std::string& text)
	{
		if (text.empty())
		{
			return std::wstring();
		}

		int size = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.c_str(), (int)text.size(), NULL, 0);
		UINT code_page = CP_UTF8;
		DWORD flags = MB_ERR_INVALID_CHARS;
		if (size == 0)
		{
			code_page = CP_ACP;
			flags = 0;
			size = MultiByteToWideChar(code_page, flags, text.c_str(), (int)text.size(), NULL, 0);
		}
		if (size == 0)
		{
			return std::wstring();
		}

		std::wstring wide(size, L'\0');
		MultiByteToWideChar(code_page, flags, text.c_str(), (int)text.size(), &wide[0], size);
		return wide;
	}

	std::string WideToCodePage(const std::wstring& text, UINT code_page)
	{
		if (text.empty())
		{
			return std::string();
		}

		int size = WideCharToMultiByte(code_page, 0, text.c_str(), (int)text.size(), NULL, 0, NULL, NULL);
		if (size == 0)
		{
			return std::string();
		}

		std::string result(size, '\0');
		WideCharToMultiByte(code_page, 0, text.c_str(), (int)text.size(), &result[0], size, NULL, NULL);
		return result;
	}

	std::string WideToUtf8(const std::wstring& text)
	{
		return WideToCodePage(text, CP_UTF8);
	}

	std::string Utf8ToLocalString(const std::string& text)
	{
		return WideToCodePage(Utf8ToWide(text), CP_ACP);
	}

	bool FileExists(const std::wstring& path)
	{
		DWORD attrs = GetFileAttributesW(path.c_str());
		return attrs != INVALID_FILE_ATTRIBUTES && (attrs & FILE_ATTRIBUTE_DIRECTORY) == 0;
	}

	bool IsAbsolutePath(const std::wstring& path)
	{
		if (path.size() >= 2 && path[1] == L':')
		{
			return true;
		}
		if (path.size() >= 2 && path[0] == L'\\' && path[1] == L'\\')
		{
			return true;
		}
		return false;
	}

	std::wstring DirectoryName(const std::wstring& path)
	{
		size_t pos = path.find_last_of(L"\\/");
		if (pos == std::wstring::npos)
		{
			return std::wstring();
		}
		return path.substr(0, pos);
	}

	std::wstring JoinPath(const std::wstring& dir, const std::wstring& name)
	{
		if (dir.empty())
		{
			return name;
		}
		const wchar_t last = dir[dir.size() - 1];
		if (last == L'\\' || last == L'/')
		{
			return dir + name;
		}
		return dir + L"\\" + name;
	}

	bool ResolveConfigPath(const std::wstring& file_name, std::wstring& out_path)
	{
		if (IsAbsolutePath(file_name))
		{
			if (FileExists(file_name))
			{
				out_path = file_name;
				return true;
			}
			return false;
		}

		std::vector<std::wstring> candidates;
		wchar_t current_dir[MAX_PATH] = { 0 };
		if (GetCurrentDirectoryW(MAX_PATH, current_dir) > 0)
		{
			std::wstring dir = current_dir;
			for (int i = 0; i < 4 && !dir.empty(); ++i)
			{
				candidates.push_back(JoinPath(dir, file_name));
				dir = DirectoryName(dir);
			}
		}

		wchar_t exe_path[MAX_PATH] = { 0 };
		if (GetModuleFileNameW(NULL, exe_path, MAX_PATH) > 0)
		{
			std::wstring dir = DirectoryName(exe_path);
			for (int i = 0; i < 4 && !dir.empty(); ++i)
			{
				candidates.push_back(JoinPath(dir, file_name));
				dir = DirectoryName(dir);
			}
		}

		for (size_t i = 0; i < candidates.size(); ++i)
		{
			if (FileExists(candidates[i]))
			{
				out_path = candidates[i];
				return true;
			}
		}

		return false;
	}

	bool ReadAllBytes(const std::wstring& path, std::vector<unsigned char>& bytes)
	{
		FILE* file = NULL;
		if (_wfopen_s(&file, path.c_str(), L"rb") != 0 || file == NULL)
		{
			return false;
		}

		fseek(file, 0, SEEK_END);
		const long length = ftell(file);
		fseek(file, 0, SEEK_SET);
		if (length < 0)
		{
			fclose(file);
			return false;
		}

		bytes.resize((size_t)length);
		if (length > 0)
		{
			const size_t read_count = fread(&bytes[0], 1, (size_t)length, file);
			fclose(file);
			return read_count == (size_t)length;
		}

		fclose(file);
		return true;
	}

	std::string BytesToText(const std::vector<unsigned char>& bytes)
	{
		if (bytes.size() >= 3 && bytes[0] == 0xEF && bytes[1] == 0xBB && bytes[2] == 0xBF)
		{
			return std::string((const char*)&bytes[3], (const char*)&bytes[0] + bytes.size());
		}

		if (bytes.size() >= 2 && bytes[0] == 0xFF && bytes[1] == 0xFE)
		{
			const size_t wchar_count = (bytes.size() - 2) / sizeof(wchar_t);
			std::wstring wide(wchar_count, L'\0');
			if (wchar_count > 0)
			{
				std::memcpy(&wide[0], &bytes[2], wchar_count * sizeof(wchar_t));
			}
			return WideToUtf8(wide);
		}

		if (bytes.empty())
		{
			return std::string();
		}
		return std::string((const char*)&bytes[0], (const char*)&bytes[0] + bytes.size());
	}

	bool ParseIniText(const std::string& text, IniMap& values)
	{
		std::string section;
		std::string pending_key;
		std::string pending_value;

		size_t start = 0;
		while (start <= text.size())
		{
			size_t end = text.find('\n', start);
			if (end == std::string::npos)
			{
				end = text.size();
			}

			std::string line = text.substr(start, end - start);
			if (!line.empty() && line[line.size() - 1] == '\r')
			{
				line.erase(line.size() - 1);
			}

			line = Trim(line);
			if (!line.empty() && line[0] != ';' && line[0] != '#')
			{
				if (line[0] == '[' && line[line.size() - 1] == ']')
				{
					CommitPendingIniValue(values, pending_key, pending_value);
					section = ToLowerAscii(Trim(line.substr(1, line.size() - 2)));
				}
				else
				{
					size_t eq = line.find('=');
					if (eq != std::string::npos && !section.empty())
					{
						CommitPendingIniValue(values, pending_key, pending_value);
						std::string key = ToLowerAscii(Trim(line.substr(0, eq)));
						std::string value = StripInlineComment(line.substr(eq + 1));
						if (value.size() >= 2 && value[0] == '"' && value[value.size() - 1] == '"')
						{
							value = value.substr(1, value.size() - 2);
						}
						pending_key = section + "." + key;
						pending_value = value;
					}
					else if (!pending_key.empty())
					{
						std::string value = StripInlineComment(line);
						if (!value.empty())
						{
							if (!pending_value.empty())
							{
								pending_value += ' ';
							}
							pending_value += value;
						}
					}
				}
			}

			if (end == text.size())
			{
				break;
			}
			start = end + 1;
		}

		CommitPendingIniValue(values, pending_key, pending_value);
		return true;
	}

	bool GetRequired(const IniMap& values, const char* section, const char* key, std::string& out)
	{
		std::string lookup = ToLowerAscii(std::string(section) + "." + key);
		IniMap::const_iterator it = values.find(lookup);
		if (it == values.end())
		{
			std::cerr << "[Config] Missing key: " << section << "." << key << std::endl;
			return false;
		}
		out = it->second;
		return true;
	}

	bool ReadString(const IniMap& values, const char* section, const char* key, std::string& out)
	{
		std::string value;
		if (!GetRequired(values, section, key, value))
		{
			return false;
		}
		out = Utf8ToLocalString(value);
		return true;
	}

	bool ReadDouble(const IniMap& values, const char* section, const char* key, double& out)
	{
		std::string value;
		if (!GetRequired(values, section, key, value))
		{
			return false;
		}
		char* end = NULL;
		const double parsed = std::strtod(value.c_str(), &end);
		if (end == value.c_str() || !Trim(end).empty())
		{
			std::cerr << "[Config] Invalid number: " << section << "." << key << std::endl;
			return false;
		}
		out = parsed;
		return true;
	}

	bool ReadFloat(const IniMap& values, const char* section, const char* key, float& out)
	{
		double value = 0.0;
		if (!ReadDouble(values, section, key, value))
		{
			return false;
		}
		out = (float)value;
		return true;
	}

	bool ReadInt(const IniMap& values, const char* section, const char* key, int& out)
	{
		std::string value;
		if (!GetRequired(values, section, key, value))
		{
			return false;
		}
		char* end = NULL;
		const long parsed = std::strtol(value.c_str(), &end, 10);
		if (end == value.c_str() || !Trim(end).empty())
		{
			std::cerr << "[Config] Invalid integer: " << section << "." << key << std::endl;
			return false;
		}
		out = (int)parsed;
		return true;
	}

	bool ParseDoubles(const std::string& text, int expected_count, std::vector<double>& values)
	{
		values.clear();
		const char* p = text.c_str();
		while (*p != '\0')
		{
			while (*p != '\0' && (std::isspace((unsigned char)*p) || *p == ','))
			{
				++p;
			}
			if (*p == '\0')
			{
				break;
			}

			char* end = NULL;
			const double value = std::strtod(p, &end);
			if (end == p)
			{
				return false;
			}
			values.push_back(value);
			p = end;
		}

		return (int)values.size() == expected_count;
	}

	bool ReadMat(const IniMap& values, const char* section, const char* key, int rows, int cols, cv::Mat& out)
	{
		std::string value;
		if (!GetRequired(values, section, key, value))
		{
			return false;
		}

		std::vector<double> parsed;
		if (!ParseDoubles(value, rows * cols, parsed))
		{
			std::cerr << "[Config] Invalid matrix: " << section << "." << key
				      << ", expected " << rows * cols << " values" << std::endl;
			return false;
		}

		out = cv::Mat(rows, cols, CV_64F);
		for (int r = 0; r < rows; ++r)
		{
			for (int c = 0; c < cols; ++c)
			{
				out.at<double>(r, c) = parsed[r * cols + c];
			}
		}

		return true;
	}
}

AppConfig::AppConfig()
{
	paths.left_images = "D:/3 Data/11 LanYou_S1/Scan_Register/Hik/L/*.bmp";
	paths.right_images = "D:/3 Data/11 LanYou_S1/Scan_Register/Hik/R/*.bmp";
	paths.template_points = "D:/3 Data/4 Track_Match/template-3D-ALL-Shift-Cut-Cut.txt";
	geo_hash.angle_tolerance_deg = 2.0f;
	geo_hash.length_tolerance = 1.0f;
	geo_hash.min_percent = 0.5f;
	geo_hash.max_distance = 650.0f;
	geo_hash.min_distance = 30.0f;

	const double scan_rt[] = {
		0.885836, -0.148476, 0.439601, 50.9177,
		0.386134, 0.76123, -0.520989, 10.9951,
		-0.257284, 0.631256, 0.731657, 45.9531,
		0.0, 0.0, 0.0, 1.0
	};
	geo_hash.scan_to_marker_RT = MakeMat(4, 4, scan_rt);

	const double I1[] = {
		5087.38336027676, -0.192267427461892, 2726.61941219956,
		0.0, 5087.34062020824, 1810.98588469888,
		0.0, 0.0, 1.0
	};
	const double D1[] = {
		-0.0702045578700941, 0.202350751405422,
		-0.00132557635846505, -0.00192989589530177,
		-0.160346625626101
	};
	const double E1[] = {
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	};
	const double I2[] = {
		5093.24802351001, 0.441691058213351, 2744.46802947870,
		0.0, 5094.80200503649, 1804.52277105108,
		0.0, 0.0, 1.0
	};
	const double D2[] = {
		-0.0685482057480179, 0.152376738914731,
		-0.000997971598550234, 0.00178921790070958,
		-0.0235569405002490
	};
	const double E2[] = {
		0.933585955134356, -0.00892689969382629, 0.358242340933799, -579.734489965219,
		-0.0141290960057581, 0.997995388314918, 0.0616893309107566, -13.3160375525600,
		-0.358074898620291, -0.0626539333482992, 0.931588348796857, 127.994051287127,
		0.0, 0.0, 0.0, 1.0
	};

	recon.I1 = MakeMat(3, 3, I1);
	recon.D1 = MakeMat(1, 5, D1);
	recon.E1 = MakeMat(4, 4, E1);
	recon.I2 = MakeMat(3, 3, I2);
	recon.D2 = MakeMat(1, 5, D2);
	recon.E2 = MakeMat(4, 4, E2);
	recon.epipolar_threshold = 15.5;
	recon.min_z_range = 1200.0f;
	recon.max_z_range = 10000.0f;
	recon.max_reproj_err = 5.5;
	recon.max_ratio = 0.7;

	detector.pyramid_levels = 3;
	detector.min_area = 100;
	detector.max_area = 7000;
	detector.ROI_w = 800;
	detector.ROI_h = 800;
	detector.perimeter_radius_px = 5.0;
	detector.min_circularity = 0.7;
	detector.intensity_threshold = 50;
	detector.debscan_min_pts = 5;

	limits.mark_point_size_max = 150;
	limits.debscan_filter_dist_max = 500.0f;
	limits.vote_pnt_size_max = 40;
	limits.vote_filter_pnt_size_min = 4;
	limits.recon_neighbor_count_min = 3;
	limits.pose_ransac_inlier_dist = 1.0f;
	limits.pose_ransac_iterations = 200;
}

AppConfig& AppConfig::Instance()
{
	static AppConfig config;
	return config;
}

const std::wstring& AppConfig::loaded_path() const
{
	return loaded_path_;
}

bool AppConfig::Load(const wchar_t* file_name)
{
	std::wstring path;
	if (!ResolveConfigPath(file_name, path))
	{
		std::cerr << "[Config] Cannot find config file." << std::endl;
		return false;
	}

	std::vector<unsigned char> bytes;
	if (!ReadAllBytes(path, bytes))
	{
		std::cerr << "[Config] Cannot read config file." << std::endl;
		return false;
	}

	IniMap values;
	if (!ParseIniText(BytesToText(bytes), values))
	{
		std::cerr << "[Config] Cannot parse config file." << std::endl;
		return false;
	}

	bool ok = true;
	ok = ReadString(values, "Paths", "left_images", paths.left_images) && ok;
	ok = ReadString(values, "Paths", "right_images", paths.right_images) && ok;
	ok = ReadString(values, "Paths", "template_points", paths.template_points) && ok;
	ok = ReadFloat(values, "GeoHash", "angle_tolerance_deg", geo_hash.angle_tolerance_deg) && ok;
	ok = ReadFloat(values, "GeoHash", "length_tolerance", geo_hash.length_tolerance) && ok;
	ok = ReadFloat(values, "GeoHash", "min_percent", geo_hash.min_percent) && ok;
	ok = ReadFloat(values, "GeoHash", "max_distance", geo_hash.max_distance) && ok;
	ok = ReadFloat(values, "GeoHash", "min_distance", geo_hash.min_distance) && ok;
	ok = ReadMat(values, "GeoHash", "scan_to_marker_RT", 4, 4, geo_hash.scan_to_marker_RT) && ok;

	ok = ReadMat(values, "Recon", "I1", 3, 3, recon.I1) && ok;
	ok = ReadMat(values, "Recon", "D1", 1, 5, recon.D1) && ok;
	ok = ReadMat(values, "Recon", "E1", 4, 4, recon.E1) && ok;
	ok = ReadMat(values, "Recon", "I2", 3, 3, recon.I2) && ok;
	ok = ReadMat(values, "Recon", "D2", 1, 5, recon.D2) && ok;
	ok = ReadMat(values, "Recon", "E2", 4, 4, recon.E2) && ok;
	ok = ReadDouble(values, "Recon", "epipolar_threshold", recon.epipolar_threshold) && ok;
	ok = ReadFloat(values, "Recon", "min_z_range", recon.min_z_range) && ok;
	ok = ReadFloat(values, "Recon", "max_z_range", recon.max_z_range) && ok;
	ok = ReadDouble(values, "Recon", "max_reproj_err", recon.max_reproj_err) && ok;
	ok = ReadDouble(values, "Recon", "max_ratio", recon.max_ratio) && ok;

	ok = ReadInt(values, "Detector", "pyramid_levels", detector.pyramid_levels) && ok;
	ok = ReadInt(values, "Detector", "min_area", detector.min_area) && ok;
	ok = ReadInt(values, "Detector", "max_area", detector.max_area) && ok;
	ok = ReadInt(values, "Detector", "ROI_w", detector.ROI_w) && ok;
	ok = ReadInt(values, "Detector", "ROI_h", detector.ROI_h) && ok;
	ok = ReadDouble(values, "Detector", "perimeter_radius_px", detector.perimeter_radius_px) && ok;
	ok = ReadDouble(values, "Detector", "min_circularity", detector.min_circularity) && ok;
	ok = ReadInt(values, "Detector", "intensity_threshold", detector.intensity_threshold) && ok;
	ok = ReadInt(values, "Detector", "debscan_min_pts", detector.debscan_min_pts) && ok;

	ok = ReadInt(values, "Limits", "mark_point_size_max", limits.mark_point_size_max) && ok;
	ok = ReadFloat(values, "Limits", "debscan_filter_dist_max", limits.debscan_filter_dist_max) && ok;
	ok = ReadInt(values, "Limits", "vote_pnt_size_max", limits.vote_pnt_size_max) && ok;
	ok = ReadInt(values, "Limits", "vote_filter_pnt_size_min", limits.vote_filter_pnt_size_min) && ok;
	ok = ReadInt(values, "Limits", "recon_neighbor_count_min", limits.recon_neighbor_count_min) && ok;
	ok = ReadFloat(values, "Limits", "pose_ransac_inlier_dist", limits.pose_ransac_inlier_dist) && ok;
	ok = ReadInt(values, "Limits", "pose_ransac_iterations", limits.pose_ransac_iterations) && ok;

	if (!ok)
	{
		return false;
	}

	if (limits.mark_point_size_max <= 0 || limits.mark_point_size_max > 255)
	{
		std::cerr << "[Config] Limits.mark_point_size_max must be in 1..255." << std::endl;
		return false;
	}
	if (limits.vote_pnt_size_max <= 0)
	{
		std::cerr << "[Config] Limits.vote_pnt_size_max must be positive." << std::endl;
		return false;
	}
	if (limits.vote_filter_pnt_size_min < 3)
	{
		std::cerr << "[Config] Limits.vote_filter_pnt_size_min must be at least 3." << std::endl;
		return false;
	}
	if (limits.recon_neighbor_count_min < 0)
	{
		std::cerr << "[Config] Limits.recon_neighbor_count_min must be non-negative." << std::endl;
		return false;
	}
	if (limits.pose_ransac_inlier_dist <= 0.0f)
	{
		std::cerr << "[Config] Limits.pose_ransac_inlier_dist must be positive." << std::endl;
		return false;
	}
	if (limits.pose_ransac_iterations <= 0)
	{
		std::cerr << "[Config] Limits.pose_ransac_iterations must be positive." << std::endl;
		return false;
	}
	if (detector.pyramid_levels < 0)
	{
		std::cerr << "[Config] Detector.pyramid_levels must be non-negative." << std::endl;
		return false;
	}
	if (detector.min_area <= 0 || detector.max_area <= detector.min_area)
	{
		std::cerr << "[Config] Detector area range is invalid." << std::endl;
		return false;
	}
	if (detector.min_circularity <= 0.0 || detector.min_circularity > 1.0)
	{
		std::cerr << "[Config] Detector.min_circularity must be in (0, 1]." << std::endl;
		return false;
	}
	if (geo_hash.min_distance <= 0.0f || geo_hash.max_distance <= geo_hash.min_distance)
	{
		std::cerr << "[Config] GeoHash distance range is invalid." << std::endl;
		return false;
	}
	if (geo_hash.angle_tolerance_deg <= 0.0f || geo_hash.angle_tolerance_deg >= 180.0f)
	{
		std::cerr << "[Config] GeoHash.angle_tolerance_deg must be in (0, 180)." << std::endl;
		return false;
	}
	if (geo_hash.length_tolerance <= 0.0f)
	{
		std::cerr << "[Config] GeoHash.length_tolerance must be positive." << std::endl;
		return false;
	}
	if (recon.min_z_range >= recon.max_z_range)
	{
		std::cerr << "[Config] Recon z range is invalid." << std::endl;
		return false;
	}

	loaded_path_ = path;
	return true;
}
