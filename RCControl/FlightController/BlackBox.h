// BlackBox

String ftos(float val){
  unsigned int  precision = 10000;
  String res = String(int(val)) + "."; // print the decimal point
  unsigned int frac;
  if(val >= 0)
    frac = (val - int(val)) * precision;
  else
    frac = (int(val)- val ) * precision;
  int frac1 = frac;
  while( frac1 /= 10 )
    precision /= 10;
  precision /= 10;
  while(  precision /= 10)
    res = res + "0";
  return res + String(frac);
}

class BlackBox{
	bool _init;	
public:
	BlackBox(){ _init = false; }
	void Init(){
		if (SD.begin(A1)) {
    		_init = true;
  		}
  		Log("started\n");
	}

	void Log(const String string){
		if (!_init) return;
		File dataFile = SD.open("BlackBox.txt", FILE_WRITE);
		if (dataFile) {
			dataFile.print(string);
			dataFile.close();
		}
	}
/*
void Error(const char* format, ...)
{
    char dest[1024 * 16];
    va_list argptr;
    va_start(argptr, format);
    vsprintf(dest, format, argptr);
    va_end(argptr);
    printf(dest);
}
void Error(const char* format, ...)
{
    va_list argptr;
    va_start(argptr, format);
    vfprintf(stderr, format, argptr);
    va_end(argptr);
}*/
};
