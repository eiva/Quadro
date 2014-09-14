#pragma once
// Led information display
class LedInfo{
 Port _r, _g, _b, _w;
public:
  LedInfo():
	  _r(GPIOB, GPIO_Pin_5),
	  _g(GPIOB, GPIO_Pin_6),
	  _b(GPIOB, GPIO_Pin_7),
	  _w(GPIOB, GPIO_Pin_8){}

  void R(bool on){
    _r.State(on);
  }
  void G(bool on){
    _g.State(on);
  }
  void B(bool on){
    _b.State(on);
  }
  void W(bool on){
    _w.State(on);
  }
  void RGBW(bool r, bool g, bool b, bool w){
    R(r); G(g); B(b); W(w);
  }
  void Off(){
	  R(false); G(false); B(false); W(false);
  }
};
