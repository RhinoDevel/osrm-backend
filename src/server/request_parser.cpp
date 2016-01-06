#include "server/request_parser.hpp"

#include "server/http/compression_type.hpp"
#include "server/http/header.hpp"
#include "server/http/request.hpp"

#include "util/tribool.hpp"

#include <boost/algorithm/string/predicate.hpp>

#include <string>

namespace osrm
{
namespace server
{

namespace http
{

RequestParser::RequestParser()
    : state(internal_state::method_start), current_header({"", ""}),
      selected_compression(no_compression), is_post_header(false), content_length(0)
{
}

std::tuple<util::tribool, compression_type>
RequestParser::parse(http::request &current_request, char *begin, char *end)
{
    while (begin != end)
    {
        util::tribool result = consume(current_request, *begin++);
        if (result != util::tribool::indeterminate)
        {
            return std::make_tuple(result, selected_compression);
        }
    }
    util::tribool result = util::tribool::indeterminate;

    if (state == internal_state::post_request && content_length <= 0)
    {
        result = util::tribool::yes;
    }
    return std::make_tuple(result, selected_compression);
}

osrm::tribool RequestParser::consume(http::request &current_request, const char input)
{
    switch (state)
    {
    case internal_state::method_start:
        if (!is_char(input) || is_CTL(input) || is_special(input))
        {
            return util::tribool::no;
        }
        if (input == 'P')
        {
            state = internal_state::post_O;
            return util::tribool::indeterminate;
        }
        state = internal_state::method;
        return util::tribool::indeterminate;
    case internal_state::post_O:
        if (input == 'O')
        {
            state = internal_state::post_S;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::post_S:
        if (input == 'S')
        {
            state = internal_state::post_T;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::post_T:
        if (input == 'T')
        {
            is_post_header = true;
            state = internal_state::method;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::post_request:
        current_request.uri.push_back(input);
        --content_length;
        return util::tribool::indeterminate;
    case internal_state::method:
        if (input == ' ')
        {
            state = internal_state::uri;
            return util::tribool::indeterminate;
        }
        if (!is_char(input) || is_CTL(input) || is_special(input))
        {
            return util::tribool::no;
        }
        return util::tribool::indeterminate;
    case internal_state::uri_start:
        if (is_CTL(input))
        {
            return util::tribool::no;
        }
        state = internal_state::uri;
        current_request.uri.push_back(input);
        return util::tribool::indeterminate;
    case internal_state::uri:
        if (input == ' ')
        {
            state = internal_state::http_version_h;
            return util::tribool::indeterminate;
        }
        if (is_CTL(input))
        {
            return util::tribool::no;
        }
        current_request.uri.push_back(input);
        return util::tribool::indeterminate;
    case internal_state::http_version_h:
        if (input == 'H')
        {
            state = internal_state::http_version_t_1;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::http_version_t_1:
        if (input == 'T')
        {
            state = internal_state::http_version_t_2;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::http_version_t_2:
        if (input == 'T')
        {
            state = internal_state::http_version_p;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::http_version_p:
        if (input == 'P')
        {
            state = internal_state::http_version_slash;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::http_version_slash:
        if (input == '/')
        {
            state = internal_state::http_version_major_start;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::http_version_major_start:
        if (is_digit(input))
        {
            state = internal_state::http_version_major;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::http_version_major:
        if (input == '.')
        {
            state = internal_state::http_version_minor_start;
            return util::tribool::indeterminate;
        }
        if (is_digit(input))
        {
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::http_version_minor_start:
        if (is_digit(input))
        {
            state = internal_state::http_version_minor;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::http_version_minor:
        if (input == '\r')
        {
            state = internal_state::expecting_newline_1;
            return util::tribool::indeterminate;
        }
        if (is_digit(input))
        {
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::expecting_newline_1:
        if (input == '\n')
        {
            state = internal_state::header_line_start;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::header_line_start:
        if (boost::iequals(current_header.name, "Accept-Encoding"))
        {
            /* giving gzip precedence over deflate */
            if (boost::icontains(current_header.value, "deflate"))
            {
                selected_compression = deflate_rfc1951;
            }
            if (boost::icontains(current_header.value, "gzip"))
            {
                selected_compression = gzip_rfc1952;
            }
        }

        if (boost::iequals(current_header.name, "Referer"))
        {
            current_request.referrer = current_header.value;
        }

        if (boost::iequals(current_header.name, "User-Agent"))
        {
            current_request.agent = current_header.value;
        }
        if (boost::iequals(current_header.name, "Content-Length"))
        {
            try
            {
                content_length = std::stoi(current_header.value);
            }
            catch (const std::exception &e)
            {
                // Ignore the header if the parameter isn't an int
            }
        }
        if (boost::iequals(current_header.name, "Content-Type"))
        {
            if (!boost::icontains(current_header.value, "application/x-www-form-urlencoded"))
            {
                return util::tribool::no;
            }
        }

        if (input == '\r')
        {
            state = internal_state::expecting_newline_3;
            return util::tribool::indeterminate;
        }
        if (!is_char(input) || is_CTL(input) || is_special(input))
        {
            return util::tribool::no;
        }
        state = internal_state::header_name;
        current_header.clear();
        current_header.name.push_back(input);
        return util::tribool::indeterminate;
    case internal_state::header_lws:
        if (input == '\r')
        {
            state = internal_state::expecting_newline_2;
            return util::tribool::indeterminate;
        }
        if (input == ' ' || input == '\t')
        {
            return util::tribool::indeterminate;
        }
        if (is_CTL(input))
        {
            return util::tribool::no;
        }
        state = internal_state::header_value;
        return util::tribool::indeterminate;
    case internal_state::header_name:
        if (input == ':')
        {
            state = internal_state::space_before_header_value;
            return util::tribool::indeterminate;
        }
        if (!is_char(input) || is_CTL(input) || is_special(input))
        {
            return util::tribool::no;
        }
        current_header.name.push_back(input);
        return util::tribool::indeterminate;
    case internal_state::space_before_header_value:
        if (input == ' ')
        {
            state = internal_state::header_value;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::header_value:
        if (input == '\r')
        {
            state = internal_state::expecting_newline_2;
            return util::tribool::indeterminate;
        }
        if (is_CTL(input))
        {
            return util::tribool::no;
        }
        current_header.value.push_back(input);
        return util::tribool::indeterminate;
    case internal_state::expecting_newline_2:
        if (input == '\n')
        {
            state = internal_state::header_line_start;
            return util::tribool::indeterminate;
        }
        return util::tribool::no;
    case internal_state::expecting_newline_3:
        if (input == '\n')
        {
            if (is_post_header)
            {
                if (content_length > 0)
                {
                    current_request.uri.push_back('?');
                }
                state = internal_state::post_request;
                return util::tribool::indeterminate;
            }
            return util::tribool::yes;
        }
        return util::tribool::no;
    default: // should never be reached
        return input == '\n' ? util::tribool::yes : util::tribool::no;
    }
}

bool RequestParser::is_char(const int character) const
{
    return character >= 0 && character <= 127;
}

bool RequestParser::is_CTL(const int character) const
{
    return (character >= 0 && character <= 31) || (character == 127);
}

bool RequestParser::is_special(const int character) const
{
    switch (character)
    {
    case '(':
    case ')':
    case '<':
    case '>':
    case '@':
    case ',':
    case ';':
    case ':':
    case '\\':
    case '"':
    case '/':
    case '[':
    case ']':
    case '?':
    case '=':
    case '{':
    case '}':
    case ' ':
    case '\t':
        return true;
    default:
        return false;
    }
}

bool RequestParser::is_digit(const int character) const
{
    return character >= '0' && character <= '9';
}
}
}
}
