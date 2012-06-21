% Create test data using:
%
% @misc{vincentsSFMToolbox,
%  Author = {Vincent Rabaud},
%  Title = {Vincent's {S}tructure from {M}otion {T}oolbox},
%  howpublished = {\url{http://vision.ucsd.edu/~vrabaud/toolbox/}}
% }
% The previous home for that project was on http://vision.ucsd.edu/~vrabaud/toolbox/doc/
% 

anim=generateToyAnimation( 0,'nPoint',5,'nFrame',3);
anim2cvyaml('rnd_N5_F3.yml',anim);
