# string.h

```cpp
#pragma once
#include<iostream>
#include<cassert>




namespace bit
{


    class string
    {
    public:

        typedef char* iterator;//迭代器
        typedef const char* const_iterator;

        iterator begin()
        {
            return _str;
        }
        const_iterator begin() const
        {
            return _str;
        }

        iterator end()
        {
            return _str + _size;
        }
        const_iterator end() const
        {
            return _str + _size;
        }
        //string()
        //    :_size(0)
        //    ,_capacity(0)
        //    ,_str(new char[1])
        //{
        //    _str[0] = '\0';
        //}

        //初始化列表要注意定义变量的顺序
        string(const char* str = "")
        :_size(strlen(str))
        ,_capacity(_size)
        ,_str(new char[_capacity + 1])
        {
            //strcpy(_str, str);
            memcpy(_str, str, _size + 1);
        }
        string(const string& str, size_t pos, size_t len = npos)
        {

            size_t length = str._size;
            //assert(pos < length);
            if (pos >= length)
            {
                _size = strlen("");
                _capacity = _size;
                _str = new char[_capacity + 1];
                _str[0] = '\0';
                return;
            }

            //if (pos + len >= length || len == npos)
            //{
            //    _size = length - pos;
            //    _capacity = _size;
            //    _str = new char[_capacity + 1];
            //    strcpy(_str, str.c_str() + pos);
            //}
            //else
            //{
            //    _size = len;
            //    _capacity = _size;
            //    _str = new char[_capacity + 1];
            //    for (int i = 0; i < _size; i++)
            //    {
            //        _str[i] = str[pos+i];
            //    }
            //    _str[_size] = '\0';
            //}

            size_t n = len;
            if (pos + len >= length || len == npos)
            {
                n = length - pos;
            }

            _size = n;
            _capacity = _size;
            _str = new char[_capacity + 1];
            for (int i = 0; i < _size; i++)
            {
                _str[i] = str[pos + i];
            }
            _str[_size] = '\0';
        }

        string(const string& str)
        :_size(str._size)
        , _capacity(str._capacity)
        , _str(new char[_capacity + 1])
        {
            //strcpy(_str, str.c_str());
            memcpy(_str, str._str, str._size+1);
        }

        ~string()
        {
            delete[] _str;
            _str = nullptr;
            _size = _capacity = 0;
        }

        const char* c_str() const
        {
            return _str;
        }

        size_t size() const
        {
            return _size;
        }

        size_t capacity() const
        {
            return _capacity;
        }

        char& operator[](size_t pos)
        {
            assert(pos >= 0 && pos < size());
            return _str[pos];
        }
        //char& operator[] (size_t pos) const
        //{
        //    assert(pos >= 0 && pos < size());
        //    return _str[pos];
        //}
        const char& operator[] (size_t pos) const
        {
            assert(pos >= 0 && pos < size());
            return _str[pos];
        }

        void reserve(size_t n = 0)
        {
            if (n > _capacity)
            {
                char* tmp = new char[n + 1];
                //strcpy(tmp, _str);
                memcpy(tmp, _str, _size);
                delete[] _str;
                _str = tmp;
                _capacity = n;
            }
        }

        void resize(size_t n, char ch = '\0')
        {
            if (n > _capacity)
            {
                reserve(n);
                for (size_t i = _size; i < n; i++)
                {
                    _str[i] = ch;
                }
                _str[n] = '\0';
                _size = n;
            }
            if (n < _size)
            {
                //erase(n);
                _str[n] = '\0';
                _size = n;
            }
        }

        void clear()
        {
            _size = 0;
            _str[0] = '\0';
        }

        void push_back(char c)
        {
            if (_size >= _capacity)
            {
                //二倍扩容
                size_t newcapacity = _capacity == 0 ? 4 : _capacity * 2;
                reserve(newcapacity);
            }
            _str[_size] = c;
            _size++;
            _str[_size] = '\0';
        }
        void append(const char* str)
        {
            size_t len = strlen(str);
            if (_size + len >= _capacity)
            {
                //至少扩容到_size+len
                reserve(_size + len);
            }
            //const char* tmp = str;
            //while(*tmp != '\0')
            //{
            //    _str[_size] = *tmp;
            //    _size++;
            //    tmp++;
            //}
            //_str[_size] = '\0';

            //strcpy(_str + _size, str);
            memcpy(_str + _size, str, len+1);
            _size += len;


        }

        string& operator+=(char c)
        {
            push_back(c);
            return *this;
        }
        string& operator+=(const char* str)
        {
            append(str);
            return *this;
        }

        void insert(size_t pos, size_t n, char c)
        {
            assert(pos <= _size);

            if (_size + n >= _capacity)
            {
                //至少扩容到_size+len
                reserve(_size + n);
            }

            //挪动数据
            //pos = 0 时有问题
            //这里不能改成int,整形提升（范围小->范围大，有符号->无符号）
            //size_t end = _size;
            //while (end >= pos)
            //{
            //    _str[end + n] = _str[end];
            //    end--;
            //}

            size_t end = _size;
            for (size_t i = _size - pos + 1; i > 0; i--)
            {
                _str[end + n] = _str[end];
                end--;
            }

            _size = _size + n;
            for (int i = 0; i < n; i++)
            {
                _str[pos + i] = c;
            }
        }
        void insert(size_t pos, const char* str)
        {
            assert(pos <= _size);
            size_t len = strlen(str);
            if (_size + len >= _capacity)
            {
                //至少扩容到_size+len
                reserve(_size + len);
            }
            size_t end = _size;
            for (size_t i = _size - pos + 1; i > 0; i--)
            {
                _str[end + len] = _str[end];
                end--;
            }
            _size = _size + len;
            for (int i = 0; i < len; i++)
            {
                _str[pos + i] = str[i];
            }

        }

        void erase(size_t pos = 0, size_t len = npos)
        {
            assert(pos <= _size);
            if (len == npos || pos + len >= _size)
            {
                _str[pos] = '\0';
                _size = pos;
            }
            else
            {
                size_t end = pos + len;
                while (end <= _size)
                {
                    _str[end - len] = _str[end];
                    end++;
                }
                _size -= len;
            }
        }

        size_t find(char c, size_t pos = 0) const
        {
            if (pos < _size)
            {
                for (int i = 0; i < _size - pos; i++)
                {
                    if (_str[pos + i] == c)
                    {
                        return pos + i;
                    }
                }
            }
            return npos;
        }

        size_t find(const char* str, size_t pos = 0) const
        {
            //one
            //size_t len = strlen(str);
            //if (pos < _size)
            //{
            //    for (int i = 0; i < _size - pos; i++)
            //    {
            //        if (_str[pos + i] == str[0])
            //        {
            //            int j = 0;
            //            for (; j < len; j++)
            //            {
            //                if (pos + i + j >= _size)
            //                {
            //                    break;
            //                }
            //                if (_str[pos + i + j] != str[j])
            //                {
            //                    break;
            //                }
            //            }
            //            if (j == len)
            //            {
            //                return pos + i;
            //            }
            //        }
            //    }
            //}
            //return npos;
            //two
            if (pos < _size)
            {
                const char* ptr = strstr(_str + pos, str);
                if (ptr != NULL)
                {
                    return ptr - _str;
                }
            }
            return npos;
        }
        string substr(size_t pos = 0, size_t len = npos) const
        {
            string str(*this, pos, len);
            return str;
        }

        bool operator<(const string& s) const
        {
            //size_t len = _size;
            //if (_size > s.size())
            //{
            //    len = s.size();
            //}
            ////return memcmp(_str, s.c_str(), len+1) < 0;
            //if (memcmp(_str, s.c_str(), len) == 0)
            //{
            //    if (_size < s.size())
            //    {
            //        return true;
            //    }
            //    else
            //    {
            //        return false;
            //    }
            //}
            //return memcmp(_str, s.c_str(), len) < 0;
            int ret = memcmp(_str, s._str, _size < s._size ? _size : s._size);
            return ret == 0 ? (_size < s.size()) : ret < 0;
        }
        bool operator==(const string& s) const
        {
            //size_t len = _size;
            //if (_size > s.size())
            //{
            //    len = s.size();
            //}
            //return (memcmp(_str, s.c_str(), len) == 0) && (_size == s.size());
            return _size == s._size && memcmp(_str, s._str, _size) == 0;
        }
        bool operator!=(const string& s) const
        {
            return !(*this == s);
        }
        bool operator<=(const string& s) const
        {
            return (*this == s) || (*this < s);
        }
        bool operator>(const string& s) const
        {
            return !(*this <= s);
        }
        bool operator>=(const string& s) const
        {
            return (*this > s) || (*this == s);
        }

        //string& operator=(const string& s)
        //{
        //    if (this != &s)
        //    {
        //        char* tmp = new char[s._capacity + 1];
        //        memcpy(tmp, s._str, s._size + 1);
        //        delete[] _str;
        //        _str = tmp;
        //        _size = s._size;
        //        _capacity = s._capacity;
        //    }
        //    return *this;
        //}

        void swap(string& s)
        {
            std::swap(_str, s._str);
            std::swap(_size, s._size);
            std::swap(_capacity, s._capacity);
        }

        //string& operator=(const string& s)
        //{
        //    if (this != &s)
        //    {
        //        string tmp(s);
        //        //std::swap(tmp, *this); 无穷递归，应为swap里面又会调operator=

        //        //std::swap(_str, tmp._str);
        //        //std::swap(_size, tmp._size);
        //        //std::swap(_capacity, tmp._capacity);

        //        //this->swap(tmp);
        //        swap(tmp);
        //    }
        //    return *this;
        //}

        string& operator=(string tmp)
        {
            swap(tmp);
            return *this;
        }
        static const size_t npos;
    private:


        size_t _size;
        size_t _capacity;
        char* _str;


    };
    const size_t string::npos = -1;
};


std::ostream& operator<<(std::ostream& out, const bit::string& s)
{
    //out << s.c_str();
    for (size_t i = 0; i < s.size(); i++)
    {
        out << s[i];
    }
    return out;
}

std::istream& operator>>(std::istream& in, bit::string& s)
{
    char ch;
    s.clear();
    char buff[128] = { '\0' };
    int i = 0;
    in >> ch;
    while (ch != ' ' && ch != '\n')
    {
        //s += ch;
        buff[i++] = ch;
        if (i == 127)
        {
            buff[i] = '\0';
            s += buff;
            i = 0;
        }
        in.get(ch);
    }
    if (i != 0)
    {
        buff[i] = '\0';
        s += buff;
    }
    return in;
}
```
