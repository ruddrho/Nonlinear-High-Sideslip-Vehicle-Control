function Fy = fiala_tire(alpha,Ca,muFz)
%FIALA_TIRE Saturating pure-slip Fiala brush tyre model.
% Sign convention: positive alpha produces negative lateral tyre force.
t = tan(alpha);
alpha_sl = atan(3*muFz/Ca);
Fy = zeros(size(alpha));
lin = abs(alpha) < alpha_sl;
tl = t(lin);
Fy(lin) = -Ca.*tl + (Ca.^2/(3*muFz)).*abs(tl).*tl ...
          - (Ca.^3/(27*muFz^2)).*tl.^3;
Fy(~lin) = -muFz.*sign(alpha(~lin));
end
