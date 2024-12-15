
#include <string.h>
#include <stdint.h>

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void DrawGlyphUnit(int x, int y, int glyph_unit_width, int glyph_unit_height, int image_width, int image_height, uint32_t color, uint32_t* buffer)
{
    for (int v = 0; v < glyph_unit_height; ++v)
    {
    int col = y + v;
    if ((col < 0) || (col >= image_height)) { continue; }
    for (int u = 0; u < glyph_unit_width; ++u)
    {
    int row = x + u;
    if ((row < 0) || (row >= image_width )) { continue; }
    buffer[col * image_width + row] = color;
    }
    }
}

// OK
static void DrawGlyphH(int x, int y, int glyph_width, int glyph_unit_width, int glyph_unit_height, int image_width, int image_height, uint32_t color, uint32_t* buffer)
{
    for (int u = glyph_unit_width; u < (glyph_width - glyph_unit_width); u += glyph_unit_width) { DrawGlyphUnit(x + u, y, glyph_unit_width, glyph_unit_height, image_width, image_height, color, buffer); }
}

// OK
static void DrawGlyphV(int x, int y, int glyph_height, int glyph_unit_width, int glyph_unit_height, int image_width, int image_height, uint32_t color, uint32_t* buffer)
{
    for (int v = glyph_unit_height; v < (glyph_height - glyph_unit_height); v += glyph_unit_height) { DrawGlyphUnit(x, y + v, glyph_unit_width, glyph_unit_height, image_width, image_height, color, buffer); }
}

// OK
static void DrawGlyphTile(int segments, int x, int y, int glyph_width, int glyph_height, int glyph_unit_width, int glyph_unit_height, int image_width, int image_height, uint32_t color, uint32_t* buffer)
{
    if (segments & 1) { DrawGlyphH(x,                                  y,                                    glyph_width,  glyph_unit_width, glyph_unit_height, image_width, image_height, color, buffer); }
    if (segments & 2) { DrawGlyphV(x + glyph_width - glyph_unit_width, y,                                    glyph_height, glyph_unit_width, glyph_unit_height, image_width, image_height, color, buffer); }
    if (segments & 4) { DrawGlyphH(x,                                  y + glyph_height - glyph_unit_height, glyph_width,  glyph_unit_width, glyph_unit_height, image_width, image_height, color, buffer); }
    if (segments & 8) { DrawGlyphV(x,                                  y,                                    glyph_height, glyph_unit_width, glyph_unit_height, image_width, image_height, color, buffer); }
}

// OK
static void DrawGlyph(int segments, int x, int y, int glyph_width, int glyph_height, int glyph_unit_width, int glyph_unit_height, int image_width, int image_height, uint32_t color, uint32_t* buffer)
{
    DrawGlyphTile(segments,      x, y,                                   glyph_width, glyph_height, glyph_unit_width, glyph_unit_height, image_width, image_height, color, buffer);
    DrawGlyphTile(segments >> 4, x, y + glyph_height - glyph_unit_width, glyph_width, glyph_height, glyph_unit_width, glyph_unit_height, image_width, image_height, color, buffer);
}

// OK
static int DigitToSegments(char number)
{
    switch (number)
    {
    case '0': return 0xEB;
    case '1': return 0x22;
    case '2': return 0xD7;
    case '3': return 0x77;
    case '4': return 0x2E;
    case '5': return 0x7D;
    case '6': return 0xF9;
    case '7': return 0x23;
    case '8': return 0xFF;
    case '9': return 0x2F;
    default:  return 0x40;
    }
}

// OK
static int DigitToSegments(wchar_t number)
{
    switch (number)
    {
    case L'0': return 0xEB;
    case L'1': return 0x22;
    case L'2': return 0xD7;
    case L'3': return 0x77;
    case L'4': return 0x2E;
    case L'5': return 0x7D;
    case L'6': return 0xF9;
    case L'7': return 0x23;
    case L'8': return 0xFF;
    case L'9': return 0x2F;
    default:   return 0x40;
    }
}

// OK
void DrawDigits(wchar_t const* str, int x, int y, int glyph_width, int glyph_kerning, int glyph_height, int glyph_unit_width, int glyph_unit_height, int image_width, int image_height, uint32_t color, uint32_t* buffer)
{
    int offset = glyph_width + glyph_kerning;   
    for (int i = 0; i < wcslen(str); ++i) { DrawGlyph(DigitToSegments(str[i]), x + (i * offset), y, glyph_width, glyph_height, glyph_unit_width, glyph_unit_height, image_width, image_height, color, buffer); }
}

// OK
void DrawDigits(char const* str, int x, int y, int glyph_width, int glyph_kerning, int glyph_height, int glyph_unit_width, int glyph_unit_height, int image_width, int image_height, uint32_t color, uint32_t* buffer)
{
    int offset = glyph_width + glyph_kerning;
    for (int i = 0; i < strlen(str); ++i) { DrawGlyph(DigitToSegments(str[i]), x + (i * offset), y, glyph_width, glyph_height, glyph_unit_width, glyph_unit_height, image_width, image_height, color, buffer); }
}
