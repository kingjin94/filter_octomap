#include <assert.h>
#include <Eigen/Dense>
#include <stack> 
#include <unordered_map>
#include <unordered_set>

#ifdef DEBUG
void resetMarker() {
	table_marker;
	table_marker.points.clear();
	table_marker.header.frame_id = "world";
	table_marker.ns = "my_namespace";
	table_marker.action = visualization_msgs::Marker::ADD;
	table_marker.pose.orientation.x = 0.0;
	table_marker.pose.orientation.y = 0.0;
	table_marker.pose.orientation.z = 0.0;
	table_marker.pose.orientation.w = 1.0; 
	table_marker.pose.position.x = 0.0;
	table_marker.pose.position.y = 0.0;
	table_marker.pose.position.z = 0.0;
	table_marker.scale.x = 0.01;
	table_marker.scale.y = 0.01;
	table_marker.scale.z = 0.01;
	table_marker.color.a = 1.0; // Don't forget to set the alpha!
	table_marker.color.r = 1.0;
	table_marker.color.g = 0.0;
	table_marker.color.b = 0.0;
	table_marker.lifetime  = ros::Duration(10.);
}
#endif


template <typename T> int sgn(T val) { // https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
    return (T(0) < val) - (val < T(0));
}

struct Index3D {
	size_t x; size_t y; size_t z;
	
	Index3D(size_t x, size_t y, size_t z) // Explicit ctor to use emplace
        : x(x), y(y), z(z) 
    {}
    
    friend std::ostream & operator << (std::ostream &out, const Index3D& i) // For printing to commandline
	{
		out << i.x << "," << i.y << "," << i.z;
		return out;
	}
    
    inline bool operator==(const Index3D& other) const { // For usage in unordered_map
        return (this->x == other.x) && (this->y == other.y) && (this->z == other.z);
    }
    
    // Neighbour indices when looking along the x axis wit z to the top
    Index3D right() const{
		return Index3D(this->x, this->y-1, this->z);
	}
	
	Index3D left() const{
		return Index3D(this->x-1, this->y+1, this->z);
	}
	
	Index3D behind() const{
		return Index3D(this->x+1, this->y, this->z);
	}
	
	Index3D before() const{
		return Index3D(this->x-1, this->y, this->z);
	}
	
	Index3D bellow() const{
		return Index3D(this->x, this->y, this->z-1);
	}
	
	Index3D above() const{
		return Index3D(this->x, this->y, this->z+1);
	}
};

namespace std { // Hash function for Index3D
    template<>
    struct hash<Index3D> {
        inline size_t operator()(const Index3D& x) const {
            return x.x ^ x.y ^ x.z;
        }
    };
}

typedef Index3D VoxelIndex;
typedef Index3D Size3D;
typedef std::unordered_map<VoxelIndex, double> VoxelindexToScore;
typedef Eigen::Vector3d Point3D;
typedef Eigen::Vector3d Vector3D;

template <class T> class Array3D { // from: https://stackoverflow.com/questions/2178909/how-to-initialize-3d-array-in-c
    size_t m_width, m_height, m_here, m_size;
    std::vector<T> m_data;
  public:
    Array3D(size_t x, size_t y, size_t z, T init = 0):
      m_width(x), m_height(y), m_data(x*y*z, init), m_here(0), m_size(x*y*z)
    {}
    Array3D(Size3D size, T init = 0):
      Array3D(size.x, size.y, size.z, init)
    {}
    T& operator()(size_t x, size_t y, size_t z) {
		assert(x < m_width);
		assert(y < m_height);
		assert(x*y*z < m_size);
        return m_data.at(x + y * m_width + z * m_width * m_height);
    }
    T& operator ()(Index3D i) {
		return (*this)(i.x, i.y, i.z);
	}
    T& operator()(size_t index) {
		return m_data.at(index);
	}
    T& operator()() {
		return m_data.at(m_here);
	}
	T& operator++() {
		m_here++;
		m_here%=(m_width * m_height);
		return m_data.at(m_here);
	}
};

typedef std::tuple<double, u_int8_t, u_int8_t, u_int8_t> RGBVoxel;
typedef Array3D<RGBVoxel> RGBVoxelgrid; // Each voxel with occupancy probability and RGB color channels
typedef Array3D<float> ScoreVoxelgrid;
typedef std::unordered_set<VoxelIndex> VoxelList;

void floodfill(VoxelindexToScore& candidates, const VoxelIndex start, VoxelList& object) {
	// Puts all indices from candidates into object if they are connected to start via the 6 Neighbourhood; removes moved indices from candidates
	#ifdef DEBUG_COUT
	std::cout << "Starting at " << start.x << "," << start.y << "," << start.z << "\n";
	#endif
	std::stack<VoxelIndex> posToLookAt;
	posToLookAt.emplace(start);
	while(!posToLookAt.empty()) {
		VoxelIndex here = posToLookAt.top();
		#ifdef DEBUG_COUT
		std::cout << "\nStack size: " << posToLookAt.size() << "\n";
		std::cout << "Looking at " << here.x << "," << here.y << "," << here.z << "\n";
		#endif
		posToLookAt.pop();
		if(candidates.find(here)!=candidates.end()) { // Still in candidate list, not yet considered
			object.insert(here);
			candidates.erase(here);
			// Test neighbours
			// y
			if(candidates.find(here.right())!=candidates.end())
				posToLookAt.emplace(here.right());
			if(candidates.find(here.left())!=candidates.end())
				posToLookAt.emplace(here.left());
			// x
			if(candidates.find(here.behind())!=candidates.end())
				posToLookAt.emplace(here.behind());
			if(candidates.find(here.before())!=candidates.end())
				posToLookAt.emplace(here.before());
			// z
			if(candidates.find(here.bellow())!=candidates.end())
				posToLookAt.emplace(here.bellow());
			if(candidates.find(here.above())!=candidates.end())
				posToLookAt.emplace(here.above());
		}
		else { // Was already removed from candidates via another route --> ignore
			continue;
		}
	}
}

VoxelIndex getMax(VoxelindexToScore& candidates) {
	// returns the key in candidates that has the bigges value (here: score)
	double biggest = -1.;
	VoxelIndex ret(0,0,0); 
	for (auto& it: candidates) {
		if(it.second > biggest) {
			biggest = it.second;
			ret = it.first;
		}
	}
	return ret;
}

void changeToGrid(octomap::ColorOcTree* octomap, RGBVoxelgrid& grid, 
				octomath::Vector3 min, octomath::Vector3 max) {
	size_t i; double x;
	for(i=0, x=min.x(); x <= max.x(); i++, x += STEP_SIZE) // over x
    {
		//#ifdef DEBUG_COUT
		//std::cout << "\ny     | x: ";
		//std::cout << x;
		//std::cout << "\n";
		//#endif
		size_t j; double y;
        for(j=0, y=min.y(); y <= max.y(); j++, y+= STEP_SIZE) // over y
        {		
			//#ifdef DEBUG_COUT	
			//std::cout << std::setw(6) << y;
			//std::cout << ": ";
			//#endif
			size_t k; double z;
            for(k=0, z=min.z(); z <= max.z(); k++, z+= STEP_SIZE) // over z
            {
				if(!octomap->search(x, y, z)) { // if place never initilized
					grid(i,j,k) = std::make_tuple(0., 0, 0, 0); // don't know anything
				} else { // already known -> just remember the log odds
					grid(i,j,k) = std::make_tuple(octomap->search(x, y, z)->getLogOdds(), 
												octomap->search(x, y, z)->getColor().r,
												octomap->search(x, y, z)->getColor().g,
												octomap->search(x, y, z)->getColor().b);
				}
				//#ifdef DEBUG_COUT
				//if(canScore(grid, {i,j,k}) > THRESHOLD_CAN)
					//std::cout << "+";
				//else 
					//std::cout << "o";
				//std::cout << " ";
				//#endif
            }
            //#ifdef DEBUG_COUT
            //std::cout << "\n";
            //#endif
        }
    }
}

void filterBoundary(VoxelList& object, VoxelList& boundary, RGBVoxelgrid& map) {
	std::cout << "Object points: " << object.size() << std::endl;
	for(VoxelIndex it: object) {
		//std::cout << "Looking at " << it << "; neighbors: ";
		//std::cout << std::get<0>(map(it.left())) << "," << std::get<0>(map(it.right())) << "," << std::get<0>(map(it.before())) << "," << std::get<0>(map(it.behind())) << ",";
		bool has_left_neighbour = (object.find(it.left()) != object.end());
		bool has_right_neighbour = (object.find(it.right()) != object.end());
		bool has_before_neighbour = (object.find(it.before()) != object.end());
		bool has_behind_neighbour = (object.find(it.behind()) != object.end());
		if(has_left_neighbour && has_right_neighbour && has_before_neighbour && has_behind_neighbour) {
			//std::cout << "skip\n";
			continue;
		}
		else
			boundary.insert(it);
	}
	std::cout << "Boundary points: " << boundary.size() << std::endl;
}

void generateCandidates(RGBVoxelgrid& map, VoxelindexToScore& candidates, const Size3D& size, std::function<double(RGBVoxelgrid&, VoxelIndex, const Size3D&)> Scorer, double threshold) {
	// Looks at all positions in map (of size size) and notes those positions where canScore is bigger threshold into the candidate map
	for(size_t i = 0; i < size.x; i++) // over x
    {
		for(size_t j = 0; j < size.y; j++) // over y
        {
            for(size_t k = 1; k < size.z; k++) // over z (0 ignored as full of table artifacts)
            {
				float score = Scorer(map, {i,j,k}, size);
				if(score > threshold) {
					//here = VoxelIndex(i,j,k);
					candidates.emplace(VoxelIndex(i,j,k), score);
				}
			}
		}
	}
}
