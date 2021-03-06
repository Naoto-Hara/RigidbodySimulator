#pragma once
#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>
#include <Eigen/Dense>
#include <vector>

namespace cereal
{
    //Eigenのデータの中身を配列として出力するためのラッパ
    template <class T>
    struct DataWrapper
    {
        DataWrapper(T& m) : mat(m) {}
        T& mat;
        template <class Archive>
        void save(Archive& ar) const
        {
            ar(cereal::make_size_tag(mat.size()));
            for (int iter = 0, end = static_cast<int>(mat.size()); iter != end; ++iter)
                ar(*(mat.data() + iter));
        }

        template <class Archive>
        void load(Archive& ar)
        {
            cereal::size_type n_rows;
            ar(cereal::make_size_tag(n_rows));
            for (int iter = 0, end = static_cast<int>(mat.size()); iter != end; ++iter)
                ar(*(mat.data() + iter));
        }
    };

    template <class Archive, class Derived, cereal::traits::DisableIf<cereal::traits::is_text_archive<Archive>::value> = cereal::traits::sfinae>
    inline void save(Archive& ar, Eigen::PlainObjectBase<Derived> const& m) {
        using ArrT = Eigen::PlainObjectBase<Derived>;
        if constexpr (ArrT::RowsAtCompileTime == Eigen::Dynamic) ar(m.rows());
        if constexpr (ArrT::ColsAtCompileTime == Eigen::Dynamic) ar(m.cols());
        ar(binary_data(m.data(), m.size() * sizeof(typename Derived::Scalar)));
    }

    template <class Archive, class Derived, cereal::traits::DisableIf<cereal::traits::is_text_archive<Archive>::value> = cereal::traits::sfinae>
    inline void load(Archive& ar, Eigen::PlainObjectBase<Derived>& m) {
        using ArrT = Eigen::PlainObjectBase<Derived>;
        Eigen::Index rows = ArrT::RowsAtCompileTime, cols = ArrT::ColsAtCompileTime;
        if constexpr (ArrT::RowsAtCompileTime == Eigen::Dynamic) ar(rows);
        if constexpr (ArrT::ColsAtCompileTime == Eigen::Dynamic) ar(cols);
        m.resize(rows, cols);
        ar(binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(typename Derived::Scalar))));
    }

}