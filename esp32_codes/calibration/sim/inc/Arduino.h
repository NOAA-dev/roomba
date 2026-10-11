#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <deque>
#include <algorithm>
#include <cstring>
#include <type_traits>
#define IRAM_ATTR
#define INPUT_PULLUP 2
#define CHANGE 3
#define F(x) (x)
#define digitalPinToInterrupt(p) (p)
template<class T,class A,class B> T constrain_(T v,A lo,B hi){return v<(T)lo?(T)lo:(v>(T)hi?(T)hi:v);}
#define constrain(v,lo,hi) constrain_((v),(lo),(hi))
class String {
 public:
  std::string s;
  String(){} String(const char*c):s(c){} String(const std::string&x):s(x){}
  size_t length() const {return s.size();}
  void trim(){size_t a=s.find_first_not_of(" \t\r\n"); if(a==std::string::npos){s="";return;} size_t b=s.find_last_not_of(" \t\r\n"); s=s.substr(a,b-a+1);}
  int indexOf(char c,int from=0) const {size_t p=s.find(c,from); return p==std::string::npos?-1:(int)p;}
  String substring(int a) const {return String(s.substr(a));}
  String substring(int a,int b) const {return String(s.substr(a,b-a));}
  double toDouble() const {return atof(s.c_str());}
  void toUpperCase(){for(auto&c:s)c=toupper(c);}
  bool equalsIgnoreCase(const char*o) const {std::string t=s,u=o;for(auto&c:t)c=toupper(c);for(auto&c:u)c=toupper(c);return t==u;}
  bool operator==(const char*o) const {return s==o;}
  String& operator+=(char c){s+=c;return *this;}
};
struct SerialT {
  std::deque<char> in; std::string out; bool echo=false;
  void begin(long){} int available(){return in.size();} char read(){char c=in.front();in.pop_front();return c;}
  template<class T> void print(T v){ char b[64]; if constexpr(std::is_floating_point<T>::value) snprintf(b,64,"%.2f",(double)v); else if constexpr(std::is_integral<T>::value) snprintf(b,64,"%ld",(long)v); else {out+=v; return;} out+=b;}
  void print(double v,int d){char b[64];snprintf(b,64,"%.*f",d,v);out+=b;}
  void println(double v,int d){print(v,d);out+="\n";}
  void print(const char*c){out+=c;} void print(String s){out+=s.s;}
  template<class T> void println(T v){print(v);out+="\n";}
  void println(){out+="\n";}
  void feed(const char*c){for(;*c;++c)in.push_back(*c);in.push_back('\n');}
} Serial;
extern unsigned long g_ms; extern unsigned long g_us;
inline unsigned long millis(){return g_ms;} inline unsigned long micros(){return g_us;}
inline void delay(int){} inline void noInterrupts(){} inline void interrupts(){}
extern int pinState[64]; extern void (*isrs[64])();
inline int digitalRead(int p){return pinState[p];}
inline void pinMode(int,int){}
inline void attachInterrupt(int p,void(*f)(),int){isrs[p]=f;}
inline void ledcSetup(int,int,int){} inline void ledcAttachPin(int,int){}
extern int duty[8];
inline void ledcWrite(int ch,int d){duty[ch]=d;}
