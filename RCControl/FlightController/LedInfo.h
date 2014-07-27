/**********************************************************************
Led information display
***********************************************************************/

class LedInfo{
 uint8_t _r, _g, _b, _w;
public:
  LedInfo():_r(3),_g(2),_b(4), _w(A3){}
  void Init(){
    pinMode(_r, OUTPUT);
    pinMode(_g, OUTPUT);
    pinMode(_b, OUTPUT);
    pinMode(_w, OUTPUT);
  }
  void R(bool on){
    digitalWrite(_r, on?HIGH:LOW);
  }
  void G(bool on){
    digitalWrite(_g, on?HIGH:LOW);
  }
  void B(bool on){
    digitalWrite(_b, on?HIGH:LOW);
  }
  void W(bool on){
    digitalWrite(_w, on?HIGH:LOW);
  }
  void RGBW(bool r, bool g, bool b, bool w){
    R(r); G(g); B(b); W(w);
  }
};
