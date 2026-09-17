#include "shoot_reference_loader.hpp"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <unordered_map>

namespace {
uint16_t U16(const std::vector<uint8_t>& bytes, size_t offset) {
    if (offset + 2 > bytes.size()) throw std::runtime_error("truncated NPZ");
    return bytes[offset] | (uint16_t(bytes[offset + 1]) << 8);
}
uint32_t U32(const std::vector<uint8_t>& bytes, size_t offset) {
    if (offset + 4 > bytes.size()) throw std::runtime_error("truncated NPZ");
    return bytes[offset] | (uint32_t(bytes[offset + 1]) << 8) |
           (uint32_t(bytes[offset + 2]) << 16) | (uint32_t(bytes[offset + 3]) << 24);
}
std::vector<uint8_t> Read(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("cannot open NPZ: " + path.string());
    stream.seekg(0, std::ios::end);
    const auto length = stream.tellg();
    stream.seekg(0, std::ios::beg);
    auto bytes = std::vector<uint8_t>(static_cast<size_t>(length));
    stream.read(reinterpret_cast<char*>(bytes.data()), length);
    return bytes;
}
std::unordered_map<std::string,std::vector<uint8_t>> Zip(const std::filesystem::path& p) {
 auto b=Read(p); size_t e=std::string::npos; for(size_t i=b.size()-4;;--i){if(U32(b,i)==0x06054b50){e=i;break;}if(i==0)break;} if(e==std::string::npos)throw std::runtime_error("invalid NPZ"); size_t c=U32(b,e+16); std::unordered_map<std::string,std::vector<uint8_t>> r;
 for(int i=0,n=U16(b,e+10);i<n;++i){if(U32(b,c)!=0x02014b50||U16(b,c+10)!=0)throw std::runtime_error("NPZ must use stored ZIP entries"); auto z=U32(b,c+24);auto nl=U16(b,c+28);auto xl=U16(b,c+30);auto cl=U16(b,c+32);auto l=U32(b,c+42);std::string name(reinterpret_cast<char*>(b.data()+c+46),nl);auto d=l+30+U16(b,l+26)+U16(b,l+28);if(d+z>b.size())throw std::runtime_error("truncated NPZ entry");r[name]={b.begin()+d,b.begin()+d+z};c+=46+nl+xl+cl;}return r;
}
struct A{std::vector<size_t>s;const uint8_t*d;size_t n;};
A Npy(const std::vector<uint8_t>& b){if(b.size()<10||std::string(reinterpret_cast<const char*>(b.data()+1),5)!="NUMPY")throw std::runtime_error("invalid NPY");size_t lb=b[6]==1?2:4,h=b[6]==1?U16(b,8):U32(b,8),o=8+lb;std::string x(reinterpret_cast<const char*>(b.data()+o),h);if(x.find("<f4")==std::string::npos||x.find("'fortran_order': True")!=std::string::npos)throw std::runtime_error("NPY must be C-order float32");std::smatch m;const std::regex shape("'shape'\\s*:\\s*\\(([^)]*)\\)");if(!std::regex_search(x,m,shape))throw std::runtime_error("NPY shape missing");A a{{},b.data()+o+h,b.size()-o-h};const std::regex dimension("[0-9]+");for(std::sregex_iterator i(m[1].first,m[1].second,dimension),z;i!=z;++i)a.s.push_back(std::stoull(i->str()));return a;}
std::vector<float> Floats(const A&a){size_t n=1;for(auto x:a.s)n*=x;if(a.n!=n*4)throw std::runtime_error("NPY size mismatch");std::vector<float>r(n);std::memcpy(r.data(),a.d,a.n);return r;}
}
std::vector<ShootReferenceData> LoadShootReferenceNpz(const std::filesystem::path& path) {
 auto z=Zip(path); auto p=Npy(z.at("joint_pos.npy")),v=Npy(z.at("joint_vel.npy")),q=Npy(z.at("body_quat_w.npy")); if(p.s.size()!=2||p.s[1]!=21||v.s!=p.s||q.s.size()!=3||q.s[0]!=p.s[0]||q.s[2]!=4||q.s[1]<1)throw std::runtime_error("invalid shoot reference arrays");auto ps=Floats(p),vs=Floats(v),qs=Floats(q);std::vector<ShootReferenceData>r;for(size_t i=0;i<p.s[0];++i){ShootReferenceData x; x.joint_position=Eigen::Map<const VecXf>(ps.data()+i*21,21);x.joint_velocity=Eigen::Map<const VecXf>(vs.data()+i*21,21);auto a=qs.data()+i*q.s[1]*4;Eigen::Quaternionf e(a[0],a[1],a[2],a[3]);if(e.norm()<1e-6f)throw std::runtime_error("invalid base quaternion");x.anchor_rotation=e.normalized().toRotationMatrix();r.push_back(std::move(x));}return r;
}
