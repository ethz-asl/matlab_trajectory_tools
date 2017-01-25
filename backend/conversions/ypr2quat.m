function q = ypr2quat(y, p, r)

cyh = cos(y/2);
syh = sin(y/2);
cph = cos(p/2);
sph = sin(p/2);
crh = cos(r/2);
srh = sin(r/2);

q = [ cyh.*cph.*crh + syh.*sph.*srh
      cyh.*cph.*srh - syh.*sph.*crh
      cyh.*sph.*crh + syh.*cph.*srh
      syh.*cph.*crh - cyh.*sph.*srh];
  
 
