#ifndef SLIC3R_GUI_BITMAP_CACHE_HPP
#define SLIC3R_GUI_BITMAP_CACHE_HPP

#include <map>
#include <vector>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif

#include "libslic3r/Color.hpp"
struct NSVGimage;

namespace Slic3r {
namespace GUI {

class BitmapCache
{
public:
	BitmapCache();
	~BitmapCache() { clear(); }
	void 			clear();
	double			scale() { return m_scale; }

	wxBitmapBundle* 	  find_bndl(const std::string &name)		{ auto it = m_bndl_map.find(name); return (it == m_bndl_map.end()) ? nullptr : it->second; }
	const wxBitmapBundle* find_bndl(const std::string &name) const	{ return const_cast<BitmapCache*>(this)->find_bndl(name); }
	wxBitmap* 		find(const std::string &name) 		{ auto it = m_map.find(name); return (it == m_map.end()) ? nullptr : it->second; }
	const wxBitmap* find(const std::string &name) const { return const_cast<BitmapCache*>(this)->find(name); }

	wxBitmapBundle*			insert_bndl(const std::string& bitmap_key, const char* data, size_t width, size_t height);
	wxBitmapBundle* 		insert_bndl(const std::string& bitmap_key, const wxBitmapBundle &bmp);
	wxBitmapBundle*			insert_bndl(const std::string& bitmap_key, const wxVector<wxBitmap>& bmps);
	wxBitmapBundle*			insert_bndl(const std::string& name, const std::vector<wxBitmapBundle*>& bmps);
	wxBitmapBundle* 		insert_raw_rgba_bndl(const std::string &bitmap_key, unsigned width, unsigned height, const unsigned char *raw_data, const bool grayscale = false);

	wxBitmap*       insert(const std::string &name, size_t width, size_t height, double scale = -1.0);
	wxBitmap* 		insert(const std::string &name, const wxBitmap &bmp);
//    wxBitmap* 		insert(const std::string &name, const wxBitmap &bmp, const wxBitmap &bmp2);
//    wxBitmap* 		insert(const std::string &name, const wxBitmap &bmp, const wxBitmap &bmp2, const wxBitmap &bmp3);
//    wxBitmap* 		insert(const std::string &name, const std::vector<wxBitmap> &bmps) { return this->insert(name, &bmps.front(), &bmps.front() + bmps.size()); }
//    wxBitmap* 		insert(const std::string &name, const wxBitmap *begin, const wxBitmap *end);
	wxBitmap* 		insert_raw_rgba(const std::string &bitmap_key, unsigned width, unsigned height, const unsigned char *raw_data, const bool grayscale = false);

	// BBS: support resize by fill border  (scale_in_center)
	// Load png from resources/icons. bitmap_key is given without the .png suffix. Bitmap will be rescaled to provided height/width if nonzero.
    wxBitmap* 		load_png(const std::string &bitmap_key, unsigned width = 0, unsigned height = 0, const bool grayscale = false, const float scale_in_center = 0.f);

	// Parses SVG file from a file, returns SVG image as paths.
	// And makes replases befor parsing
	// replace_map containes old_value->new_value
	static NSVGimage* nsvgParseFromFileWithReplace(const char* filename, const char* units, float dpi, const std::map<std::string, std::string>& replaces);
	// Gets a data from SVG file and makes replases
	// replace_map containes old_value->new_value
    static void		nsvgGetDataFromFileWithReplace(const char* filename, std::string& data_str, const std::map<std::string, std::string>& replaces);
	wxBitmapBundle* from_svg(const std::string& bitmap_name, unsigned target_width, unsigned target_height, const bool dark_mode, const std::string& new_color = "");
	wxBitmapBundle* from_png(const std::string& bitmap_name, unsigned width, unsigned height);
	// Load svg from resources/icons. bitmap_key is given without the .svg suffix. SVG will be rasterized to provided height/width.
    wxBitmap* 		load_svg(const std::string &bitmap_key, unsigned width = 0, unsigned height = 0, const bool grayscale = false, const bool dark_mode = false, const std::string& new_color = "", const float scale_in_center = 0.f);

//	wxBitmap 		mksolid(size_t width, size_t height, unsigned char r, unsigned char g, unsigned char b, unsigned char transparency, bool suppress_scaling = false, size_t border_width = 0, bool dark_mode = false);
//	wxBitmap 		mksolid(size_t width, size_t height, const unsigned char rgb[3], bool suppress_scaling = false, size_t border_width = 0, bool dark_mode = false) { return mksolid(width, height, rgb[0], rgb[1], rgb[2], wxALPHA_OPAQUE, suppress_scaling, border_width, dark_mode); }
//	wxBitmap 		mksolid(size_t width, size_t height, const ColorRGB& rgb, bool suppress_scaling = false, size_t border_width = 0, bool dark_mode = false) { return mksolid(width, height, rgb.r_uchar(), rgb.g_uchar(), rgb.b_uchar(), wxALPHA_OPAQUE, suppress_scaling, border_width, dark_mode); }
//	wxBitmap 		mkclear(size_t width, size_t height) { return mksolid(width, height, 0, 0, 0, wxALPHA_TRANSPARENT, true, 0); }
	wxBitmapBundle	mksolid(size_t width, size_t height, unsigned char r, unsigned char g, unsigned char b, unsigned char transparency, size_t border_width = 0, bool dark_mode = false);
	wxBitmapBundle*	mksolid_bndl(size_t width, size_t height, const std::string& color = std::string(), size_t border_width = 0, bool dark_mode = false);
	wxBitmapBundle* mkclear_bndl(size_t width, size_t height) { return 	mksolid_bndl(width, height); }

	static bool     parse_color(const std::string& scolor, unsigned char* rgb_out);
	static bool     parse_color4(const std::string& scolor, unsigned char* rgba_out);

private:
    std::map<std::string, wxBitmap*>	m_map;
    std::map<std::string, wxBitmapBundle*>	m_bndl_map;
    double	m_gs	= 0.2;	// value, used for image.ConvertToGreyscale(m_gs, m_gs, m_gs)
	double	m_scale = 1.0;	// value, used for correct scaling of SVG icons on Retina display
};

} // GUI
} // Slic3r

#endif /* SLIC3R_GUI_BITMAP_CACHE_HPP */
