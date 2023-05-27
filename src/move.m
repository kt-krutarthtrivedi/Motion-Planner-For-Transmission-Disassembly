function A = move(qr, qn, val, eps)
   qnew = [0 0];
   if val >= eps
       qnew(1) = round(qn(1) + ((qr(1)-qn(1))*eps)/euclidean_distance(qr,qn));
       qnew(2) = round(qn(2) + ((qr(2)-qn(2))*eps)/euclidean_distance(qr,qn));
       qnew(3) = round(qn(3) + ((qr(3)-qn(3))*eps)/euclidean_distance(qr,qn));
   else
       qnew(1) = qr(1);
       qnew(2) = qr(2);
       qnew(3) = qr(3);
   end
   
   A = [qnew(1), qnew(2), qnew(3)];
end