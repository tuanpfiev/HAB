namespace ballet
{
    int64_t EARTH_RADIUS = 6371009;
    typedef size_t ENTRY_ID;
    
    struct Mercator
    {
        double lat, lon, alt;

        Mercator() {}

        Mercator(double lat, double lon, double alt = 0)
        {
            this->lat = lat;
            this->lon = lon;
            this->alt = alt;
        }

        Mercator(hmath::Spherical spherical)
        {
            this->lat = hmath::toDegree(spherical.phi);
            this->lon = hmath::toDegree(spherical.theta);
            this->alt = spherical.r - EARTH_RADIUS;
        }

        Mercator(hmath::Vector3 position)
        {
			this->lon = hmath::toDegree(atan2(position.normed().j, position.normed().i));
			this->lat = hmath::toDegree(asin(position.normed().k));
			this->alt = position.norm() - EARTH_RADIUS;
        }

        hmath::Vector3 R3()
		{
			hmath::Vector3 v;
			v.i = (this->alt + EARTH_RADIUS) * cos(hmath::toRad(this->lon)) * sin(hmath::toRad(90 - this->lat));
			v.j = (this->alt + EARTH_RADIUS) * sin(hmath::toRad(this->lon)) * sin(hmath::toRad(90 - this->lat));
			v.k = (this->alt + EARTH_RADIUS) * cos(hmath::toRad(90 - this->lat));
			return v;
		}
    };

    std::ostream& operator<<(std::ostream& os, Mercator mercator)
	{
		os << mercator.lat << ", " << mercator.lon << ", " << mercator.alt;
		return os;
	}

    // Standard Locations

    Mercator horsham(-36.7189, 142.1962, 300);
    Mercator tullamarine(-37.6995, 144.8803, 300);  

    hmath::Vector3 localWind2UCS(hmath::Vector2 local, hmath::Vector3 position)
    {
        hmath::Vector3 new_x(hmath::Vector3(0,0,1), position, hmath::VECTOR3D_CROSS);
        hmath::Vector3 new_y(position, new_x, hmath::VECTOR3D_CROSS);
        new_x.normalize();
        new_y.normalize();

        new_x *= local.i;
        new_y *= local.j;

        hmath::Vector3 UCS_wind = new_x + new_y;

        return UCS_wind;
    }

    bool operator==(Mercator m1, Mercator m2)
    {
        if (m1.lat == m2.lat && m1.lon == m2.lon && m1.alt == m2.alt)
        {
            return true;
        }
        
        else
        {
            return false;
        }
    }

    bool operator!=(Mercator m1, Mercator m2)
    {
        if (m1.lat == m2.lat && m1.lon == m2.lon && m1.alt == m2.alt)
        {
            return false;
        }
        
        else
        {
            return true;
        }
    }

    struct Extents
    {
        public:
            Mercator bl, tr;
        
        Extents() {}

        Extents(Mercator bl, Mercator tr)
        {
            this->bl = bl;
            this->tr = tr;
        }
    };

    struct C0000
    {
        double epoch;
        Mercator C000;
    };

    class Node
    {
    private:
        hmath::Vector2 wind_vector;
        Mercator location;
        double pressure = -1, temperature = -1, density = -1;
        bool pressure_set = false, temperature_set = false;
        double dummy;

        void setDensity()
        {
            if (this->pressure_set && this->temperature_set)
            {
                this->density = (this->pressure * 100) / (this->temperature * 287.058);
            }
        }

    public:

        const static ENTRY_ID wind_i_id = 0;
        const static ENTRY_ID wind_j_id = 1;
        const static ENTRY_ID temperature_id = 3;
        const static ENTRY_ID pressure_id = 4;
        const static ENTRY_ID altitude_id = 2;
        const static ENTRY_ID latitude_id = 6;
        const static ENTRY_ID longitude_id = 7;
        const static ENTRY_ID density_id = 5;

        Node() {}
        ~Node() {}

        hmath::Vector2 getWindVector()
        {
            return this->wind_vector;
        }

        Mercator getLocation()
        {
            return this->location;
        }

        double getPressure()
        {
            return this->pressure;
        }

        double getTemperature()
        {
            return this->temperature;
        }

        double getDensity()
        {
            return this->density;
        }

        double getAltitude()
        {
            return this->location.alt;
        }

        void setWindVector(hmath::Vector2 wind_vector)
        {
            this->wind_vector = wind_vector;
        }

        void setLocation(Mercator location)
        {
            this->location = location;
        }

        void setPressure(double pressure)
        {
            this->pressure = pressure;
            this->pressure_set = true;
            this->setDensity();
        }

        void setTemperature(double temperature)
        {
            this->temperature = temperature;
            this->temperature_set = true;
            this->setDensity();
        }

        void setAltitude(double alt)
        {
            this->location.alt = alt;
        }

        void initialize()
        {
            if (!(this->temperature < 0 && this->pressure < 0 && this->density < 0))
            {
                this->pressure_set = true;
                this->temperature_set = true;

                this->setDensity();
            }
            else if (!(this->temperature < 0 && this->pressure < 0))
            {
                this->pressure_set = true;
                this->temperature_set = true;
            }
        }

        double& setEntry(const ENTRY_ID function_id)
        {
            switch (function_id)
            {
            case wind_i_id:
                return this->wind_vector.i;

            case wind_j_id:
                return this->wind_vector.j;

            case temperature_id:
                return this->temperature;

            case pressure_id:
                return this->pressure;

            case altitude_id:
                return this->location.alt;

            case latitude_id:
                return this->location.lat;

            case longitude_id:
                return this->location.lon;

            case density_id:
                return this->density;
            }

            return this->dummy;
        }
    };

    Node operator+(Node n1, Node n2)
	{
		Node new_node;

		new_node.setWindVector(n1.getWindVector() + n2.getWindVector());
		new_node.setAltitude(n1.getAltitude() + n2.getAltitude());
		new_node.setTemperature(n1.getTemperature() + n2.getTemperature());
        new_node.setPressure(n1.getPressure() + n2.getPressure());

		return new_node;
	}

	Node operator-(Node n1, Node n2)
	{
		Node new_node;

		new_node.setWindVector(n1.getWindVector() - n2.getWindVector());
		new_node.setAltitude(n1.getAltitude() - n2.getAltitude());
		new_node.setTemperature(n1.getTemperature() - n2.getTemperature());
        new_node.setPressure(n1.getPressure() - n2.getPressure());

		return new_node;
	}

    Node operator*(Node n, double s)
	{
		Node new_node;

		new_node.setWindVector(n.getWindVector() * s);
		new_node.setAltitude(n.getAltitude() * s);
		new_node.setTemperature(n.getTemperature() * s);
        new_node.setPressure(n.getPressure() * s);

		return new_node;
	}

	Node operator*(double s, Node n)
	{
		Node new_node;

		new_node.setWindVector(n.getWindVector() * s);
		new_node.setAltitude(n.getAltitude() * s);
		new_node.setTemperature(n.getTemperature() * s);
        new_node.setPressure(n.getPressure() * s);

		return new_node;
	}

    struct Frustum
    {
        Node C000, C100, C010, C110, C001, C101, C011, C111;
    };

    Frustum operator+(Frustum f1, Frustum f2)
    {
        Frustum new_frustum;

        new_frustum.C000 = f1.C000 + f2.C000;
        new_frustum.C100 = f1.C100 + f2.C100;
        new_frustum.C010 = f1.C010 + f2.C010;
        new_frustum.C110 = f1.C110 + f2.C110;
        new_frustum.C001 = f1.C001 + f2.C001;
        new_frustum.C101 = f1.C101 + f2.C101;
        new_frustum.C011 = f1.C011 + f2.C011;
        new_frustum.C111 = f1.C111 + f2.C111;

        return new_frustum;
    }

    Frustum operator-(Frustum f1, Frustum f2)
    {
        Frustum new_frustum;

        new_frustum.C000 = f1.C000 - f2.C000;
        new_frustum.C100 = f1.C100 - f2.C100;
        new_frustum.C010 = f1.C010 - f2.C010;
        new_frustum.C110 = f1.C110 - f2.C110;
        new_frustum.C001 = f1.C001 - f2.C001;
        new_frustum.C101 = f1.C101 - f2.C101;
        new_frustum.C011 = f1.C011 - f2.C011;
        new_frustum.C111 = f1.C111 - f2.C111;

        return new_frustum;
    }

    Frustum operator*(Frustum f, double s)
    {
        Frustum new_frustum;

        new_frustum.C000 = f.C000 * s;
        new_frustum.C100 = f.C100 * s;
        new_frustum.C010 = f.C010 * s;
        new_frustum.C110 = f.C110 * s;
        new_frustum.C001 = f.C001 * s;
        new_frustum.C101 = f.C101 * s;
        new_frustum.C011 = f.C011 * s;
        new_frustum.C111 = f.C111 * s;

        return new_frustum;
    }

    Frustum operator*(double s, Frustum f)
    {
        Frustum new_frustum;

        new_frustum.C000 = f.C000 * s;
        new_frustum.C100 = f.C100 * s;
        new_frustum.C010 = f.C010 * s;
        new_frustum.C110 = f.C110 * s;
        new_frustum.C001 = f.C001 * s;
        new_frustum.C101 = f.C101 * s;
        new_frustum.C011 = f.C011 * s;
        new_frustum.C111 = f.C111 * s;

        return new_frustum;
    }

    struct Hyperfrustum
    {
        Frustum t0, t1;
    };

    class Latitude
    {
    private:
        std::vector<Node> nodes;
    public:
        Latitude() {}
        ~Latitude() {}

        Latitude(std::vector<Node>& nodes, double lat)
        {
            for (Node& node : nodes)
            {
                if (node.getLocation().lat != lat)
                {
                    std::invalid_argument("Latitude of supplied nodes does not match the designated latitude. (Cross reference check)");
                }
                
                this->nodes = nodes;
            }
        }

        Node& getNodeByIndex(int idx)
        {
            if (idx == -1)
            {
                return this->nodes.back();
            }
            
            else
            {
                return this->nodes[idx];
            }
            
        }

        int numNodes()
        {
            return this->nodes.size();
        }
    };

    class Layer
    {
    private:
        std::vector<Latitude> latitudes;
        Mercator bl, tr;

    public:
        Layer() {}
        ~Layer() {}

        Layer(std::vector<Latitude>& latitudes, Mercator bl, Mercator tr)
        {
            for (Latitude& latitude : latitudes)
            {
                if (latitude.getNodeByIndex(0).getLocation().lat < bl.lat || latitude.getNodeByIndex(0).getLocation().lon < bl.lat || latitude.getNodeByIndex(-1).getLocation().lat > tr.lat || latitude.getNodeByIndex(-1).getLocation().lon > tr.lon)
                {
                    std::invalid_argument("At least one submitted node lays outside the bounds provided in construction. (Cross reference check)");
                }
                
            }
            
            this->latitudes = latitudes;
            this->bl = bl;
            this->tr = tr;
        }

        std::pair<Mercator, Mercator> getExtents()
        {
            return std::pair<Mercator, Mercator>(this->bl, this->tr);
        }

        Latitude& getLatitudeByIndex(int idx)
        {
            if (idx == -1)
            {
                return this->latitudes.back();
            }
            
            else
            {
                return this->latitudes[idx];
            }
        }

        int numLatitudes()
        {
            return this->latitudes.size();
        }
    };

    class Chunk
    {
    private:
        std::vector<Layer> layers;
        Mercator bl, tr;
        int64_t epoch;
        bool altitude_restructured = false;
        
    public:
        Chunk() {}
        ~Chunk() {}

        Chunk(std::vector<Layer>& layers, Mercator bl, Mercator tr, int64_t epoch)
        {
            for (Layer& layer : layers)
            {
                if (layer.getExtents().first == bl && layer.getExtents().second == tr)
                {
                    std::invalid_argument("The extents of at least one layer do not match the extents provided. (Cross reference check)");
                }
            }

            this->layers = layers;
            this->bl = bl;
            this->tr = tr;
            this->epoch = epoch;
        }

        std::pair<Mercator, Mercator> getExtents()
        {
            if (this->altitude_restructured)
            {
                return std::pair<Mercator, Mercator>(this->bl, this->tr);
            }
            
            else
            {
                Mercator bl, tr;

                bl.lat = this->bl.lat;
                bl.lon = this->bl.lon;
                bl.alt = 0;
                
                tr.lat = this->tr.lat;
                tr.lon = this->tr.lon;
                tr.alt = 0;

                return std::pair<Mercator, Mercator>(bl, tr);
            }  
        }

        Layer& getLayerByIndex(int idx)
        {
            if (idx == -1)
            {
                return this->layers.back();
            }
            
            else
            {
                return this->layers[idx];
            }
        }

        int numLayers()
        {
            return this->layers.size();
        }

        int64_t getEpoch()
        {
            return this->epoch;
        }

        void setAltExtents(double lower, double upper)
        {
            this->bl.alt = lower;
            this->tr.alt = upper;
        }
    };

    class Environment
    {
    private:
        std::vector<Chunk> chunks;
        Mercator bl, tr;
        int64_t t0, tn;
        std::vector<double> levels;
        double t_res, grid_res;
        bool restructured = false;
    public:
        Environment() {}
        ~Environment() {}

        Environment(std::vector<Chunk>& chunks, Mercator bl, Mercator tr, double grid_res, double t_res, int64_t t0, int64_t tn)
        {
            for (Chunk& chunk : chunks)
            {
                if (chunk.getExtents().first != bl || chunk.getExtents().second != tr)
                {
                    std::invalid_argument("The extents of at least one provided chunk does not match the extents defined. (Cross reference check)");
                }

                if (chunk.getEpoch() < t0 || chunk.getEpoch() > tn)
                {
                    std::invalid_argument("The epoch of at least one provided chunk does not match the epochs defined. (Cross reference check)");
                }                                               
            }

            if (chunks[0].getEpoch() != t0 || chunks.back().getEpoch() != tn)
            {
                std::invalid_argument("Epochs of start and end chunks do not match the provided epoch extents. (Cross reference check)");
            }
            
            this->chunks = chunks;
            this->bl = bl;
            this->tr = tr;
            this->t0 = t0;
            this->tn = tn;
            this->grid_res = grid_res;
        }

        Environment(std::string path)
        {
            std::vector<std::string> file_names = {"ugrdprs.csv", "vgrdprs.csv", "hgtprs.csv", "tmpprs.csv", "prsprs.csv", "denprs.csv"};
            std::vector<std::string> datasets;

            for (std::string file_name : file_names)
            {
                std::fstream data_stream;
                data_stream.open(path + file_name);
                std::stringstream sstr;
                sstr << data_stream.rdbuf();
                datasets.push_back(sstr.str());
            }
            
            std::ifstream meta_data;
            meta_data.open(path + "meta_data.csv");

            std::string line, entry;
            
            std::getline(meta_data, line);

            std::stringstream epoch_info(line);

            std::getline(epoch_info, entry, ',');
            this->t0 = int64_t(std::stod(entry));
            std::getline(epoch_info, entry, ',');
            this->tn = int64_t(std::stod(entry));
            std::getline(epoch_info, entry, ',');
            this->t_res = int64_t(std::stod(entry));

            std::getline(meta_data, line);

            std::stringstream level_info(line);

            while (std::getline(level_info, entry, ','))
            {
                this->levels.push_back(std::stod(entry));
            }            
            
            std::getline(meta_data, line);

            std::stringstream bl_info(line);

            std::getline(bl_info, entry, ',');
            this->bl.lat = std::stod(entry);
            std::getline(bl_info, entry, ',');
            this->bl.lon = std::stod(entry);
            std::getline(bl_info, entry, ',');
            this->bl.alt = std::stod(entry);

            std::getline(meta_data, line);

            std::stringstream tr_info(line);

            std::getline(tr_info, entry, ',');
            this->tr.lat = std::stod(entry);
            std::getline(tr_info, entry, ',');
            this->tr.lon = std::stod(entry);
            std::getline(tr_info, entry, ',');
            this->tr.alt = std::stod(entry);

            std::getline(meta_data, line);

            std::stringstream res_info(line);

            std::getline(res_info, entry);
            this->grid_res = std::stod(entry);

            std::vector<size_t> line_first_guess = {0,0,0,0,0,0};

            for (int64_t t = this->t0; t <= this->tn; t += this->t_res)
            {
                int t_idx = (t - this->t0) / this->t_res;

                std::vector<ballet::Layer> layers;

                for (int z = 0; z < this->levels.size(); z++)
                {
                    std::vector<ballet::Latitude> latitudes;

                    for (double lat = bl.lat; lat <= tr.lat; lat += this->grid_res)
                    {
                        int lat_idx = (lat - bl.lat) / this->grid_res;

                        std::string leader = "[" + std::to_string(t_idx) + "][" + std::to_string(z) + "][" + std::to_string(lat_idx) + "]";

                        std::vector<ballet::Node> nodes(((this->tr.lon - this->bl.lon) / this->grid_res) + 1);

                        for (int idx = 0; idx < datasets.size(); idx++)
                        {
                            std::string* data = &datasets[idx];
                            
                            size_t line_start = data->find(leader, line_first_guess[idx]);
                            size_t line_end = data->find("\n", line_start);
                            line_first_guess[idx] = line_end;
                            size_t data_start = data->find(", ", line_start);

                            std::string line = data->substr(data_start + 2, line_end - data_start - 2);

                            int entry_start = 0;
                            int entry_end = 0;

                            for (double lon = this->bl.lon; lon <= this->tr.lon; lon += this->grid_res)
                            {
                                size_t lon_idx = (lon - this->bl.lon) / this->grid_res;

                                entry_end = line.find(", ", entry_start);

                                nodes[lon_idx].setEntry(ballet::Node::latitude_id) = lat;
                                nodes[lon_idx].setEntry(idx) = std::stod(line.substr(entry_start, entry_end - entry_start));
                                entry_start = entry_end += 2;

                                if (idx == datasets.size() - 1)
                                {
                                    nodes[lon_idx].initialize();
                                }
                            }
                        }

                        latitudes.push_back(ballet::Latitude(nodes, lat));
                    }

                    layers.push_back(ballet::Layer(latitudes, this->bl, this->tr));
                }

                this->chunks.push_back(ballet::Chunk(layers, this->bl, this->tr, t));
            }
        }

        void constructFromChunks(std::vector<Chunk>& chunks, Mercator bl, Mercator tr, double grid_res, double t_res, int64_t t0, int64_t tn)
        {
            for (Chunk& chunk : chunks)
            {
                if (chunk.getExtents().first != bl || chunk.getExtents().second != tr)
                {
                    std::invalid_argument("The extents of at least one provided chunk does not match the extents defined. (Cross reference check)");
                }

                if (chunk.getEpoch() < t0 || chunk.getEpoch() > tn)
                {
                    std::invalid_argument("The epoch of at least one provided chunk does not match the epochs defined. (Cross reference check)");
                }                                               
            }

            if (chunks[0].getEpoch() != t0 || chunks.back().getEpoch() != tn)
            {
                std::invalid_argument("Epochs of start and end chunks do not match the provided epoch extents. (Cross reference check)");
            }
            
            this->chunks = chunks;
            this->bl = bl;
            this->tr = tr;
            this->t0 = t0;
            this->tn = tn;
            this->t_res = t_res;
            this->grid_res = grid_res;
        }

        Chunk& getChunkByIndex(int idx)
        {
            if (idx == -1)
            {
                return this->chunks.back();
            }
            
            else
            {
                return this->chunks[idx];
            }
        }

        bool inEnvironment(Mercator mercator, double epoch)
        {
            bool epoch_domain = (epoch > this->t0) && (epoch < this->tn);
            bool lat_domain = (mercator.lat > this->bl.lat) && (mercator.lat < this->tr.lat);
            bool lon_domain = (mercator.lon > this->bl.lon) && (mercator.lon < this->tr.lon);
            bool alt_domain = (mercator.alt > this->bl.alt) && (mercator.alt < this->tr.alt);

            if (lat_domain && lon_domain && alt_domain && epoch_domain)
            {
                return true;
            }

            else
            {
                return false;
            }                        
        }

        Extents getExtents()
        {
            return Extents(this->bl, this->tr);
        }

        void restructureByAltitude()
        {
			std::vector<double> level_heights;

            for (int i = 0; i < this->chunks[0].numLayers(); i++)
            {
                level_heights.push_back(0);
            }
            
			for (Chunk& chunk : this->chunks)
			{

				for (int z = 0; z < this->chunks[0].numLayers(); z++)
				{
					double total = 0;

					for (int j = 0; j < this->chunks[0].getLayerByIndex(z).numLatitudes(); j++)
					{
						for (int i = 0; i < this->chunks[0].getLayerByIndex(z).getLatitudeByIndex(j).numNodes(); i++)
						{
							total += chunk.getLayerByIndex(z).getLatitudeByIndex(j).getNodeByIndex(i).getAltitude();
						}
					}

					double avg = total / (this->chunks[0].getLayerByIndex(0).numLatitudes() * this->chunks[0].getLayerByIndex(0).getLatitudeByIndex(0).numNodes());
					level_heights[z] += avg;
				}
			}

			for (int i = 0; i < level_heights.size(); i++)
			{
				level_heights[i] /= this->chunks.size();
				this->levels.push_back(level_heights[i]);
			}

			for (Chunk& chunk : this->chunks)
			{
                chunk.setAltExtents(level_heights[0], level_heights.back());
				for (int z = 0; z < this->chunks[0].numLayers(); z++)
				{
					double total = 0;

					for (int j = 0; j < this->chunks[0].getLayerByIndex(z).numLatitudes(); j++)
					{
						for (int i = 0; i < this->chunks[0].getLayerByIndex(z).getLatitudeByIndex(j).numNodes(); i++)
						{
                            //std::cout << i << ", " << j << ", " << z << ": " << level_heights[z] << std::endl;
							chunk.getLayerByIndex(z).getLatitudeByIndex(j).getNodeByIndex(i).setAltitude(level_heights[z]);
                        }
					}
				}
			}

            this->bl.alt = level_heights[0];
            this->tr.alt = level_heights.back();

            this->restructured = true;
        }

        C0000 getSpatiotemporalBaseVertice(Mercator mercator, double epoch)
        {
            double epoch_relative = (epoch - this->t0) / this->t_res;

            Mercator relative_mercator;

            relative_mercator.lat = (mercator.lat - bl.lat) / this->grid_res;
            relative_mercator.lon = (mercator.lon - bl.lon) / this->grid_res;

            for (int idx = 0; idx < this->levels.size(); idx++)
            {
                if (this->levels[idx] > mercator.alt)
                {
                    //std::cout << idx << std::endl;
                    relative_mercator.alt = ((mercator.alt - this->levels[idx - 1]) / (this->levels[idx] - this->levels[idx - 1])) + (idx - 1);
                    break;
                }
            }

            C0000 new_spatiotemporal_base_vertice;
            new_spatiotemporal_base_vertice.C000 = relative_mercator;
            new_spatiotemporal_base_vertice.epoch = epoch_relative;

            return new_spatiotemporal_base_vertice;
        }

        Node subsampleVectorFieldForNode(Mercator mercator, double epoch)
        {
            double epoch_relative = (epoch - this->t0) / this->t_res;

            Mercator relative_mercator;

            relative_mercator.lat = (mercator.lat - bl.lat) / this->grid_res;
            relative_mercator.lon = (mercator.lon - bl.lon) / this->grid_res;

            for (int idx = 0; idx < this->levels.size(); idx++)
            {
                if (this->levels[idx] > mercator.alt)
                {
                    relative_mercator.alt = ((mercator.alt - this->levels[idx - 1]) / (this->levels[idx] - this->levels[idx - 1])) + (idx - 1);
                    break;
                }
            }

            Hyperfrustum hyperfrustum;
            hyperfrustum.t0.C000 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t0.C100 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t0.C010 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t0.C110 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t0.C001 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t0.C101 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t0.C011 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t0.C111 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t1.C000 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t1.C100 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t1.C010 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t1.C110 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t1.C001 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t1.C101 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t1.C011 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t1.C111 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));

            Frustum tif = ((hyperfrustum.t1 - hyperfrustum.t0) * (epoch_relative - floor(epoch_relative))) + hyperfrustum.t0;

            Node c000c100 = ((tif.C100 - tif.C000) * (relative_mercator.lon - floor(relative_mercator.lon))) + tif.C000;
            Node c010c110 = ((tif.C110 - tif.C010) * (relative_mercator.lon - floor(relative_mercator.lon))) + tif.C010;
            Node c001c101 = ((tif.C101 - tif.C001) * (relative_mercator.lon - floor(relative_mercator.lon))) + tif.C001;
            Node c011c111 = ((tif.C111 - tif.C011) * (relative_mercator.lon - floor(relative_mercator.lon))) + tif.C011;

            Node c000c100c010c110 = ((c010c110 - c000c100) * (relative_mercator.lat - floor(relative_mercator.lat))) + c000c100;
            Node c001c101c011c111 = ((c011c111 - c001c101) * (relative_mercator.lat - floor(relative_mercator.lat))) + c001c101;

            Node c000c100c010c110c001c101c011c111 = ((c001c101c011c111 - c000c100c010c110) * (relative_mercator.alt - floor(relative_mercator.alt))) + c000c100c010c110;

            return c000c100c010c110c001c101c011c111;          
        }

        Frustum subsampleVectorFieldForFrustum(Mercator mercator, double epoch)
        {
            double epoch_relative = (epoch - this->t0) / this->t_res;

            Mercator relative_mercator;

            relative_mercator.lat = (mercator.lat - bl.lat) / this->grid_res;
            relative_mercator.lon = (mercator.lon - bl.lon) / this->grid_res;

            for (int idx = 0; idx < this->levels.size(); idx++)
            {
                if (this->levels[idx] > mercator.alt)
                {
                    //std::cout << idx << std::endl;
                    relative_mercator.alt = ((mercator.alt - this->levels[idx - 1]) / (this->levels[idx] - this->levels[idx - 1])) + (idx - 1);
                    break;
                }
            }

            Hyperfrustum hyperfrustum;
            hyperfrustum.t0.C000 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t0.C100 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t0.C010 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t0.C110 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t0.C001 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t0.C101 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t0.C011 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t0.C111 = this->getChunkByIndex(floor(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t1.C000 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t1.C100 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t1.C010 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t1.C110 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(floor(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t1.C001 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t1.C101 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(floor(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));
            hyperfrustum.t1.C011 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(floor(relative_mercator.lon));
            hyperfrustum.t1.C111 = this->getChunkByIndex(ceil(epoch_relative)).getLayerByIndex(ceil(relative_mercator.alt)).getLatitudeByIndex(ceil(relative_mercator.lat)).getNodeByIndex(ceil(relative_mercator.lon));

            Frustum temporally_interpolated_frustum = ((hyperfrustum.t1 - hyperfrustum.t0) * (epoch_relative - floor(epoch_relative))) + hyperfrustum.t0;

            return temporally_interpolated_frustum;         
        }

        std::vector<double> getLevels()
        {
            return this->levels;
        }

        void saveToCSV(std::string path)
        {
            if (!this->restructured)
            {
                this->restructureByAltitude();
            }
            
            std::ofstream ugrdprs;
            ugrdprs.open(path + "ugrdprs.csv");
            std::ofstream vgrdprs;
            vgrdprs.open(path + "vgrdprs.csv");
            std::ofstream hgtprs;
            hgtprs.open(path + "hgtprs.csv");
            std::ofstream tmpprs;
            tmpprs.open(path + "tmpprs.csv");
            std::ofstream prsprs;
            prsprs.open(path + "prsprs.csv");
            std::ofstream denprs;
            denprs.open(path + "denprs.csv");

            for (int t = 0; t < this->chunks.size(); t++)
            {
                for (int z = 0; z < this->levels.size(); z++)
                {
                    for (int y = 0; y < this->chunks[0].getLayerByIndex(0).numLatitudes(); y++)
                    {
                        std::string header = "[" + std::to_string(t) + "][" + std::to_string(z) + "][" + std::to_string(y) + "]";

                        ugrdprs << header;
                        vgrdprs << header;
                        hgtprs << header;
                        tmpprs << header;
                        prsprs << header;
                        denprs << header;

                        for (int x = 0; x < this->chunks[0].getLayerByIndex(0).getLatitudeByIndex(0).numNodes(); x++)
                        {
                            Node& node = this->chunks[t].getLayerByIndex(z).getLatitudeByIndex(y).getNodeByIndex(x);

                            ugrdprs << ", " << std::to_string(node.getWindVector().i);
                            vgrdprs << ", " << std::to_string(node.getWindVector().j);
                            hgtprs << ", " << std::to_string(node.getAltitude());
                            tmpprs << ", " << std::to_string(node.getTemperature());
                            prsprs << ", " << std::to_string(node.getPressure());
                            denprs << ", " << std::to_string(node.getDensity());
                        }

                        ugrdprs << "\n";
                        vgrdprs << "\n";
                        hgtprs << "\n";
                        tmpprs << "\n";
                        prsprs << "\n";
                        denprs << "\n";
                    }

                    ugrdprs << "\n"; 
                    vgrdprs << "\n"; 
                    hgtprs << "\n"; 
                    tmpprs << "\n";
                    prsprs << "\n";
                    denprs << "\n";
                }

                ugrdprs << "\n"; 
                vgrdprs << "\n";
                hgtprs << "\n";
                tmpprs << "\n";
                prsprs << "\n";
                denprs << "\n";
            }

            ugrdprs.close();
            vgrdprs.close();
            hgtprs.close();
            tmpprs.close();
            prsprs.close();
            denprs.close();

            std::ofstream meta_data;
            meta_data.open(path + "meta_data.csv");

            meta_data << std::to_string(this->t0) << ", " << std::to_string(this->tn) << ", " << std::to_string(this->t_res) << "\n";
            std::string levels_line;
            for (int i = 0; i < this->levels.size(); i++)
            {                
                levels_line += ((i > 0) ? ", " : "") + std::to_string(this->levels[i]);
            }

            meta_data << levels_line;
            meta_data << "\n";
            
            meta_data << std::to_string(this->bl.lat) << ", " << std::to_string(this->bl.lon) << ", " << std::to_string(this->bl.alt) << "\n";
            meta_data << std::to_string(this->tr.lat) << ", " << std::to_string(this->tr.lon) << ", " << std::to_string(this->tr.alt) << "\n";
            meta_data << std::to_string(this->grid_res) << "\n";
        }
    };

    class Camera
    {
    private:
        double focal_length = 1;
        double pixel_pitch = 1;
        double format = 1;

    public:
        Camera() {}
        ~Camera() {}

        Camera(double focal_length, double pixel_pitch, double format)
        {
            this->focal_length = focal_length;
            this->pixel_pitch = pixel_pitch;
            this->format = format;
        }

        void setFocalLength(double focal_length)
        {
            this->focal_length = focal_length;
        }

        double getFocalLength()
        {
            return this->focal_length;
        }

        void setPixelPitch(double pixel_pitch)
        {
            this->pixel_pitch = pixel_pitch;
        }

        double getPixelPitch()
        {
            return this->pixel_pitch;
        }

        void setFormat(double format)
        {
            this->format = format;
        }

        double getFormat()
        {
            return this->format;
        }
    };
    
    class Balloon
    {
    private:
        hmath::Vector3 position, previous_position;
        double ascent_rate, burst_height;
        bool falling = false;
        Camera* camera;
        bool camera_attached = false;
    public:
        Balloon() {}
        ~Balloon() {}

        void setOrigin(Mercator location)
        {
            this->position = location.R3();
        }

        Mercator getCurrentMercator()
        {
            return Mercator(this->position);
        }

        Mercator getPreviousMercator()
        {
            return Mercator(this->previous_position);
        }

        hmath::Vector3 getCurrentR3()
        {
            return this->position;
        }

        hmath::Vector3 getPreviousR3()
        {
            return this->previous_position;
        }

        void setAscentRate(double ascent_rate)
        {
            this->ascent_rate = ascent_rate;
        }

        double getAltitude()
        {
            return this->position.norm() - EARTH_RADIUS;
        }

        void setBurstAltitude(double altitude)
        {
            this->burst_height = altitude;
        }

        void setFalling(bool falling)
        {
            this->falling = falling;
        }

        bool isFalling()
        {
            return this->falling;
        }

        void propagateForward(Node subsampled_node, double& epoch, double time_step = 1)
        {
            hmath::Vector3 UCS_wind = localWind2UCS(subsampled_node.getWindVector(), this->position);
            this->previous_position = this->position;

            if (!this->falling)
            {
                UCS_wind *= time_step;
                this->position += UCS_wind;
                this->position *= (1 + ((ascent_rate * time_step) / this->position.norm()));

                if (this->getAltitude() > this->burst_height)
                {
                    this->falling = true;
                }
            }

            else if (this->falling)
            {
                double descent_rate = 5 + exp((2 + (this->getAltitude() * this->getAltitude())) / 300000000);
                UCS_wind *= time_step;
                this->position += UCS_wind;
                this->position *= (1 - ((descent_rate * time_step) / this->position.norm()));
            }

            epoch += time_step;
        }

        void propagateBackward(Node& subsampled_node, double& epoch, double time_step = 1)
        {
            hmath::Vector3 UCS_wind = localWind2UCS(subsampled_node.getWindVector(), this->position);
            this->previous_position = this->position;

            if (!this->falling)
            {
                UCS_wind *= time_step;
                this->position -= UCS_wind;
                this->position *= (1 - ((ascent_rate * time_step) / this->position.norm()));
            }

            else if (this->falling)
            {
                double descent_rate = 5 + exp((2 + (this->getAltitude() * this->getAltitude())) / 300000000);
                UCS_wind *= time_step;
                this->position -= UCS_wind;
                this->position *= (1 + ((descent_rate * time_step) / this->position.norm()));

                if (this->getAltitude() > this->burst_height)
                {
                    this->falling = false;
                }
            }

            epoch -= time_step;
        }

        void attachCamera(Camera* camera)
        {
            this->camera = camera;
            this->camera_attached = true;
        }

        double getNadirPixelSize()
        {
            if (this->camera_attached)
            {
                return (this->getAltitude() / this->camera->getFocalLength()) * this->camera->getPixelPitch();
            }

            else
            {
                return 0;
            }
        }

        double getOrthogonalSwathWidth()
        {
            if (this->camera_attached)
            {
                return (this->getAltitude() / this->camera->getFocalLength()) * this->camera->getFormat();
            }
            
            else
            {
                return 0;
            }
        }
    };

    class KML
    {
    private:
        std::string path;
        std::ofstream output;
    public:
        KML() {}
        ~KML() {}

        KML(std::string path, std::string filename)
        {
            this->output.open(path + filename + ".kml", std::ios::out);

            this->output << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
            this->output << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
            this->output << "<Document>\n";
            this->output << "<name>Flight Path</name>\n";
            this->output << "<description><![CDATA[Simulated in the Balloon Artificial Research Environment for Navigation and Autonomy (BallARENA)]]></description>\n";
            this->output << "<Style id=\"yellowPoly\">\n";
            this->output << "<LineStyle>\n";
            this->output << "<color>7f00ffff</color>\n";
            this->output << "<width>4</width>\n";
            this->output << "</LineStyle>\n";
            this->output << "<PolyStyle>\n";
            this->output << "<color>7f00ff00</color>\n";
            this->output << "</PolyStyle>\n";
            this->output << "</Style>\n";
            this->output << "<Placemark>\n";
            this->output << "<name>Flight path</name>\n";
            this->output << "<styleUrl>#stratoLine</styleUrl>\n";
            // this->output << "<styleUrl>#yellowPoly</styleUrl>\n";
            this->output << "<LineString>\n";
            //this->output << "<extrude>1</extrude>\n";
            //this->output << "<tesselate>1</tesselate>\n";
            this->output << "<altitudeMode>absolute</altitudeMode>\n";
            this->output << "<coordinates>\n";
        }

        std::ofstream& outputFile()
        {
            return this->output;
        }

        void closeKML()
        {
            this->output << "</coordinates>\n";
            this->output << "</LineString></Placemark>";
            this->output << "</Document></kml>\n";
            this->output.close();
        }
    };
}
