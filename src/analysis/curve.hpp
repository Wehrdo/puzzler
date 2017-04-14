#ifndef CURVE_H
#define CURVE_H

enum curve_type
{
   indent,
   outdent,
   END,
};

class Curve
   {
   public:
      cv::Point origin;
      int start;
      int end;
      curve_type type;

      Curve( int start, int end, cv::Point origin, curve_type type )
         {
         this->start = start;
         this->end = end;
         this->origin = origin;
         this->type = type;
         }

      bool operator==(const Curve rhs)
         {
         return this->type == rhs.type;
         }

   };

#endif /* CURVE_H */
