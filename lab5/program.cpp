 #include <boost/numeric/ublas/vector.hpp>
 #include <boost/numeric/ublas/vector_proxy.hpp>
 #include <boost/numeric/ublas/matrix.hpp>
 #include <boost/numeric/ublas/triangular.hpp>
 #include <boost/numeric/ublas/lu.hpp>
 #include <boost/numeric/ublas/io.hpp>
 #include <boost/numeric/ublas/matrix_proxy.hpp> 
 #include <boost/qvm/mat_operations.hpp>

using namespace boost::numeric::ublas;

//KOD ZA DETERMINANTU I INVERZ JE PREUZET S INTERNETA http://www.richelbilderbeek.nl/CppUblasMatrixExample6.htm

int determinant( matrix<double> m, int n) {
   int det = 0;
   matrix<double> submatrix(5,5);
   if (n == 2){
   
      return ((m(0,0) * m(1,1) - (m(1,0) * m(0,1))));
   }

   else {
      for (int x = 0; x < n; x++) {
         int subi = 0;
         for (int i = 1; i < n; i++) {
            int subj = 0;
            for (int j = 0; j < n; j++) {
               if (j == x){
               
                   continue;
               }

               submatrix(subi,subj) = m(i,j);
               subj++;
            }
            subi++;
         }
         det = det + (pow(-1, x) * m(0,x) * determinant( submatrix, n - 1 ));
      }
   }
   return det;
}


double CalcDeterminant(const boost::numeric::ublas::matrix<double>& m)
{
  assert(m.size1() == m.size2() && "Can only calculate the determinant of square matrices");
  switch(m.size1())
  {
    case 1:
    {
      return m(0,0);
    }
    case 2:
    {
      const double a = m(0,0);
      const double b = m(0,1);
      const double c = m(1,0);
      const double d = m(1,1);
      const double determinant = (a * d) - (b * c);
      return determinant;
    }
    case 3:
    {
      assert(m.size1() == 3 && m.size2() == 3 && "Only for 3x3 matrices");
      const double a = m(0,0);
      const double b = m(0,1);
      const double c = m(0,2);
      const double d = m(1,0);
      const double e = m(1,1);
      const double f = m(1,2);
      const double g = m(2,0);
      const double h = m(2,1);
      const double k = m(2,2);
      const double determinant
        = (a * ((e*k) - (f*h)))
        - (b * ((k*d) - (f*g)))
        + (c * ((d*h) - (e*g)));
      return determinant;
    }
    default:
      assert(!"Should not get here: unsupported matrix size");
      throw std::runtime_error("Unsupported matrix size");
  }
}

///Chop returns a std::vector of sub-matrices
//[ A at [0]   B at [1] ]
//[ C at [2]   D at [4] ]
const std::vector<boost::numeric::ublas::matrix<double> > Chop(
  const boost::numeric::ublas::matrix<double>& m)
{
  using boost::numeric::ublas::range;
  using boost::numeric::ublas::matrix;
  using boost::numeric::ublas::matrix_range;
  std::vector<matrix<double> > v;
  v.reserve(4);
  const int midy = m.size1() / 2;
  const int midx = m.size2() / 2;
  const matrix_range<const matrix<double> > top_left(    m,range(0   ,midy     ),range(0   ,midx     ));
  const matrix_range<const matrix<double> > bottom_left( m,range(midy,m.size1()),range(0   ,midx     ));
  const matrix_range<const matrix<double> > top_right(   m,range(0   ,midy     ),range(midx,m.size2()));
  const matrix_range<const matrix<double> > bottom_right(m,range(midy,m.size1()),range(midx,m.size2()));
  v.push_back(matrix<double>(top_left));
  v.push_back(matrix<double>(top_right));
  v.push_back(matrix<double>(bottom_left));
  v.push_back(matrix<double>(bottom_right));
  return v;
}

const boost::numeric::ublas::matrix<double> CreateMatrix(
  const std::size_t n_rows,
  const std::size_t n_cols,
  const std::vector<double>& v)
{
  assert(n_rows * n_cols == v.size());
  boost::numeric::ublas::matrix<double> m(n_rows,n_cols);
  for (std::size_t row = 0; row!=n_rows; ++row)
  {
    for (std::size_t col = 0; col!=n_cols; ++col)
    {
      m(row,col) = v[ (col * n_rows) + row];
    }
  }
  return m;
}

const boost::numeric::ublas::matrix<double> CreateRandomMatrix(const std::size_t n_rows, const std::size_t n_cols)
{
  boost::numeric::ublas::matrix<double> m(n_rows,n_cols);
  for (std::size_t row=0; row!=n_rows; ++row)
  {
    for (std::size_t col=0; col!=n_cols; ++col)
    {
      m(row,col) = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
    }
  }
  return m;
}

///Unchop merges the 4 std::vector of sub-matrices produced by Chop
const boost::numeric::ublas::matrix<double> Unchop(
  const std::vector<boost::numeric::ublas::matrix<double> >& v)
{
  //Chop returns a std::vector of sub-matrices
  //[ A at [0]   B at [1] ]
  //[ C at [2]   D at [4] ]
  using boost::numeric::ublas::range;
  using boost::numeric::ublas::matrix;
  using boost::numeric::ublas::matrix_range;
  assert(v.size() == 4);
  assert(v[0].size1() == v[1].size1());
  assert(v[2].size1() == v[3].size1());
  assert(v[0].size2() == v[2].size2());
  assert(v[1].size2() == v[3].size2());
  boost::numeric::ublas::matrix<double> m(v[0].size1() + v[2].size1(),v[0].size2() + v[1].size2());
  for (int quadrant=0; quadrant!=4; ++quadrant)
  {
    const boost::numeric::ublas::matrix<double>& w = v[quadrant];
    const std::size_t n_rows = v[quadrant].size1();
    const std::size_t n_cols = v[quadrant].size2();
    const int offset_x = quadrant % 2 ? v[0].size2() : 0;
    const int offset_y = quadrant / 2 ? v[0].size1() : 0;
    for (std::size_t row=0; row!=n_rows; ++row)
    {
      for (std::size_t col=0; col!=n_cols; ++col)
      {
        m(offset_y + row, offset_x + col) = w(row,col);
      }
    }
  }

  assert(v[0].size1() + v[2].size1() == m.size1());
  assert(v[1].size1() + v[3].size1() == m.size1());
  assert(v[0].size2() + v[1].size2() == m.size2());
  assert(v[2].size2() + v[3].size2() == m.size2());

  return m;
}

const boost::numeric::ublas::matrix<double> Inverse(
  const boost::numeric::ublas::matrix<double>& m)
{
  assert(m.size1() == m.size2() && "Can only calculate the inverse of square matrices");

  switch(m.size1())
  {
    case 1:
    {
      assert(m.size1() == 1 && m.size2() == 1 && "Only for 1x1 matrices");
      const double determinant = CalcDeterminant(m);
      assert(determinant != 0.0);
      assert(m(0,0) != 0.0 && "Cannot take the inverse of matrix [0]");
      boost::numeric::ublas::matrix<double> n(1,1);
      n(0,0) =  1.0 / determinant;
      return n;
    }
    case 2:
    {
      assert(m.size1() == 2 && m.size2() == 2 && "Only for 2x2 matrices");
      const double determinant = CalcDeterminant(m);
      assert(determinant != 0.0);
      const double a = m(0,0);
      const double b = m(0,1);
      const double c = m(1,0);
      const double d = m(1,1);
      boost::numeric::ublas::matrix<double> n(2,2);
      n(0,0) =  d / determinant;
      n(0,1) = -b / determinant;
      n(1,0) = -c / determinant;
      n(1,1) =  a / determinant;
      return n;
    }
    case 3:
    {
      assert(m.size1() == 3 && m.size2() == 3 && "Only for 3x3 matrices");
      const double determinant = CalcDeterminant(m);
      assert(determinant != 0.0);
      const double a = m(0,0);
      const double b = m(0,1);
      const double c = m(0,2);
      const double d = m(1,0);
      const double e = m(1,1);
      const double f = m(1,2);
      const double g = m(2,0);
      const double h = m(2,1);
      const double k = m(2,2);
      boost::numeric::ublas::matrix<double> n(3,3);
      const double new_a =  ((e*k)-(f*h)) / determinant;
      const double new_b = -((d*k)-(f*g)) / determinant;
      const double new_c =  ((d*h)-(e*g)) / determinant;
      const double new_d = -((b*k)-(c*h)) / determinant;
      const double new_e =  ((a*k)-(c*g)) / determinant;
      const double new_f = -((a*h)-(b*g)) / determinant;
      const double new_g =  ((b*f)-(c*e)) / determinant;
      const double new_h = -((a*f)-(c*d)) / determinant;
      const double new_k =  ((a*e)-(b*d)) / determinant;
      n(0,0) = new_a;
      n(1,0) = new_b;
      n(2,0) = new_c;
      n(0,1) = new_d;
      n(1,1) = new_e;
      n(2,1) = new_f;
      n(0,2) = new_g;
      n(1,2) = new_h;
      n(2,2) = new_k;
      return n;
    }
    default:
    {
      //Use blockwise inversion
      //Matrix::Chop returns a std::vector
      //[ A at [0]   B at [1] ]
      //[ C at [2]   D at [4] ]
      const std::vector<boost::numeric::ublas::matrix<double> > v = Chop(m);
      const boost::numeric::ublas::matrix<double>& a = v[0];
      assert(a.size1() == a.size2());
      const boost::numeric::ublas::matrix<double>  a_inv = Inverse(a);
      const boost::numeric::ublas::matrix<double>& b = v[1];
      const boost::numeric::ublas::matrix<double>& c = v[2];
      const boost::numeric::ublas::matrix<double>& d = v[3];
      const boost::numeric::ublas::matrix<double> term
        = d
        - prod(
            boost::numeric::ublas::matrix<double>(prod(c,a_inv)),
            b
          );
      const boost::numeric::ublas::matrix<double> term_inv = Inverse(term);
      const boost::numeric::ublas::matrix<double> new_a
        = a_inv
        + boost::numeric::ublas::matrix<double>(prod(
            boost::numeric::ublas::matrix<double>(prod(
              boost::numeric::ublas::matrix<double>(prod(
                boost::numeric::ublas::matrix<double>(prod(
                  a_inv,
                  b)),
                term_inv)),
             c)),
            a_inv));

      const boost::numeric::ublas::matrix<double> new_b
        =
        - boost::numeric::ublas::matrix<double>(prod(
            boost::numeric::ublas::matrix<double>(prod(
              a_inv,
              b)),
            term_inv));

      const boost::numeric::ublas::matrix<double> new_c
        =
        - boost::numeric::ublas::matrix<double>(prod(
            boost::numeric::ublas::matrix<double>(prod(
              term_inv,
              c)),
            a_inv));

      const boost::numeric::ublas::matrix<double> new_d = term_inv;
      const std::vector<boost::numeric::ublas::matrix<double> > w = { new_a, new_b, new_c, new_d };
      const boost::numeric::ublas::matrix<double> result = Unchop(w);
      return result;
    }
  }
} 


int main () {


    matrix<double> m1 (5,5);
    matrix<double> m2 (5,5);
    vector<double> v (5);
    identity_matrix<double> m3 (5);
    
    m1(0,0) = 0;
    m1(0,1) = 0;
    m1(0,2) = 3;
    m1(0,3) = 6;
    m1(0,4) = 5;
    
    v(0) = 1;
    v(1) = 5;
    v(2) = 1;
    v(3) = 0;
    v(4) = 9;
    
    for(unsigned i = 1; i < m1.size1(); i++){
    
    	for(unsigned j = 0; j < m1.size2(); j++){
    	
    		m1(i,j) = m1(0,j); 
    		
    	}
    }	
    
    m2 = m1+m3;
    
    matrix<double> m4 = Inverse(m2); //inverz matrice
    
    std::cout<<prod(m2,v)<<std::endl; //produkt vektora i matrice
    std::cout<<inner_prod(v,trans(v))<<std::endl; //produkt vektora i transponiranog vektora
    std::cout<<determinant(m2,m2.size1()); // determinanta matrice
    std::cout<<m4;
    

}
