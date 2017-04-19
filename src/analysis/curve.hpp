#ifndef CURVE_H
#define CURVE_H


class Curve
   {
   public:
      enum curve_type
      {
         indent,
         outdent,
         END,
      };

      cv::Point origin;
      size_t start;
      size_t end;
      curve_type type;



      Curve( size_t start, size_t end, cv::Point origin, curve_type type )
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
