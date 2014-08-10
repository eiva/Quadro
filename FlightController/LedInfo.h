#pragma once
// Led information display
class LedInfo{
 Port _r, _g, _b, _w;
public:
  LedInfo():
	  _r(GPIOB, GPIO_Pin_15),
	  _g(GPIOB, GPIO_Pin_14),
	  _b(GPIOB, GPIO_Pin_13),
	  _w(GPIOB, GPIO_Pin_12){}
  void Init(){}
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
};
