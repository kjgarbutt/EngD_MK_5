package sim;

import agents.LimitedActionsAgent;
import agents.NGOAgent;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.planargraph.Node;
import ec.util.MersenneTwisterFast;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.field.geo.GeomVectorField;
import sim.io.geo.ShapeFileImporter;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraph;
import sim.util.geo.GeomPlanarGraphEdge;
import sim.util.geo.MasonGeometry;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;

/**
 *
 * A simple model that locates agents on Norfolk's road network and makes them
 * move from A to B, then they change direction and head back to the start.
 * The process repeats until the user quits. The number of agents, their start
 * and end points is determined by data in NorfolkITNLSOA.csv and assigned by the
 * user under 'goals' (approx. Line 84).
 *
 * @author KJGarbutt
 *
 */
public class MK_5 extends SimState	{

	////////////////////////////////////////////////////////////////
	///////////////////// MODEL PARAMETERS /////////////////////////
	////////////////////////////////////////////////////////////////

	private static final long serialVersionUID = -4554882816749973618L;

	//////////////////// Containers //////////////////////////////
    public GeomVectorField roads = new GeomVectorField();
    public GeomVectorField world = new GeomVectorField();
    public GeomVectorField flood3 = new GeomVectorField();
    public GeomVectorField flood2 = new GeomVectorField();
    //public GeomVectorField HouseholdsFZ = new GeomVectorField();
    //public GeomVectorField Households = new GeomVectorField();
    public GeomVectorField agents = new GeomVectorField();
    public GeomVectorField ngoagents = new GeomVectorField();
    public GeomVectorField elderlyagents = new GeomVectorField();
    public GeomVectorField limitedactionsagents = new GeomVectorField();

    ////////////////////// Network ///////////////////////////////
    public GeomPlanarGraph network = new GeomPlanarGraph();	// Stores road network connections
    public GeomVectorField junctions = new GeomVectorField();	// nodes for intersections
    
    ///////////////////// MainAgent //////////////////////////////
    // maps between unique edge IDs and edge structures themselves
    HashMap<Integer, GeomPlanarGraphEdge> idsToEdges =
        new HashMap<Integer, GeomPlanarGraphEdge>();
    public HashMap<GeomPlanarGraphEdge, ArrayList<agents.MainAgent>> edgeTraffic =
        new HashMap<GeomPlanarGraphEdge, ArrayList<agents.MainAgent>>();

    public GeomVectorField mainagents = new GeomVectorField();
    ArrayList<agents.MainAgent> agentList = new ArrayList<agents.MainAgent>();
    ArrayList<Polygon> polys = new ArrayList<Polygon>();
    
    // Here we force the agents to go to or from work at any time
    public boolean goToWork = true;
    public boolean getGoToWork()	{
        return goToWork;
    }
    
    ///////////////////// NGOAgent //////////////////////////////
    HashMap<Integer, GeomPlanarGraphEdge> idsToEdges1 =
            new HashMap<Integer, GeomPlanarGraphEdge>();
    public HashMap<GeomPlanarGraphEdge, ArrayList<NGOAgent>> edgeTraffic1 =
            new HashMap<GeomPlanarGraphEdge, ArrayList<NGOAgent>>();

    public GeomVectorField ngogents = new GeomVectorField();
    ArrayList<NGOAgent> ngoAgentList = new ArrayList<NGOAgent>();

    public boolean goToWork1 = true;
    public boolean getGoToWork1()	{
        return goToWork1;
    }

    //////////////////// ElderlyAgent ///////////////////////////
    HashMap<Integer, GeomPlanarGraphEdge> idsToEdges2 =
            new HashMap<Integer, GeomPlanarGraphEdge>();
    public HashMap<GeomPlanarGraphEdge, ArrayList<agents.ElderlyAgent>> edgeTraffic2 =
            new HashMap<GeomPlanarGraphEdge, ArrayList<agents.ElderlyAgent>>();

    public GeomVectorField elderlyagent = new GeomVectorField();
    ArrayList<agents.ElderlyAgent> elderlyAgentList = new ArrayList<agents.ElderlyAgent>();

    public boolean goToWork2 = true;
    public boolean getGoToWork2()	{
        return goToWork2;
    }

    //////////////////// LimitedActionsAgent //////////////////////
    HashMap<Integer, GeomPlanarGraphEdge> idsToEdges3 =
            new HashMap<Integer, GeomPlanarGraphEdge>();
    public HashMap<GeomPlanarGraphEdge, ArrayList<LimitedActionsAgent>> edgeTraffic3 =
            new HashMap<GeomPlanarGraphEdge, ArrayList<LimitedActionsAgent>>();

    public GeomVectorField limitedactionsagent = new GeomVectorField();
    ArrayList<LimitedActionsAgent> limitedActionsAgentList = new ArrayList<LimitedActionsAgent>();

    public boolean goToWork3 = true;
    public boolean getGoToWork3()	{
        return goToWork3;
    }
    
    //////////////////// agentGoals //////////////////////
    
    /**
     * Here we set the 'goals', or destinations, which relate to ROAD_ID in NorfolkITNLSOA.csv/.shp...
     * 30250 = Norfolk & Norwich Hospital
     * 74858 = James Paget Hospital (Does not work!)
     * 18081 = Queen Elizabeth Hospital
     * 46728 = BRC Norwich Office
     * 49307 = Sprowston Fire Station - BRC Fire & Emergency Support
     *
     */
    
    Integer[] goals =	{	// MainAgent
    		60708, 70353, 75417, 29565, 3715, 15816, 47794, 16561, 70035, 55437, 98, 45, 987, 345, 5643, 234, 21, 8765, 10345
    };
    
    Integer[] goals1 =	{	// NGOAgent
    		60708, 70353, 75417, 29565, 3715, 15816, 47794, 16561, 70035, 55437
    };

    Integer[] goals2 =	{	// ElderlyAgent
    		60708, 70353, 75417, 29565, 3715, 15816, 47794, 16561, 70035, 55437
    };

    Integer[] goals3 =	{	// LimitedActionsAgent
    		60708, 70353, 75417, 29565, 3715, 15816, 47794, 16561, 70035, 55437
    };


    ///////////////////////////////////////////////////////////////////////////
	/////////////////////////// BEGIN FUNCTIONS ///////////////////////////////
	///////////////////////////////////////////////////////////////////////////


    /**
     * Model Constructor
     */
    public MK_5(long seed)	{
        super(seed);
        random = new MersenneTwisterFast(12345);
    }
    
    void setup()
    {
        // copy over the geometries into a list of Polygons
        Bag ps = world.getGeometries();
        polys.addAll(ps);
        }

    /**
     * Model Initialization
     */
    @Override
    public void start() {
        super.start();
        //System.out.println("Working Directory = " +
                //System.getProperty("user.dir"));
        //File roadsFile1 = new File("/Volumes/JetDrive/Drive/ABM/engd/engd_mk_5/data/Final_ITN.shp");
        //System.out.println("Path : " + roadsFile1.getAbsolutePath());

        /*
        File folder = new File("/Volumes/JetDrive/Drive/ABM/engd/engd_mk_5/data");
        System.out.println("'Data'folder = " +folder);
        File[] listOfFiles = folder.listFiles();

        for (int i = 0; i < listOfFiles.length; i++) {
            if (listOfFiles[i].isFile()) {
                System.out.println("File " + listOfFiles[i].getName());
            } else if (listOfFiles[i].isDirectory()) {
                System.out.println("Directory " + listOfFiles[i].getName());
            }
        }
        */
        System.out.println("Reading shapefiles...");

		//////////////////////////////////////////////
		///////////// READING IN DATA ////////////////
		//////////////////////////////////////////////

        /*
        /Volumes/JetDrive/Drive/ABM/engd/engd_mk_5/resources/
        ABM
            engd
               engd_mk_5/
                    lib/
                    resources/
                        .shp, .dbf, .csv ...
                    src/
                        sim/
                            MK_5.java etc.
                        agents/
                            MainAgent.java etc.
         */
        
        try {

            // read in the roads shapefile to create the transit network
        	
        	URL roadsFile = MK_5.class.getResource
                     ("/Final_ITN.shp");
            ShapeFileImporter.read(roadsFile, roads);
            System.out.println("	Roads shapefile: " +roadsFile);

            Envelope MBR = roads.getMBR();
            
            // read in the LSOA shapefile to create the backgroundss
            //URL areasFile = MK_4.class.getResource
            //		("/data/Final_LSOA.shp");
	        //   Bag desiredAttributes = new Bag();
	        //   desiredAttributes.add("RC_RankCol");
            //
	        //   try {
	        //   	ShapeFileImporter.read(areasFile, world, desiredAttributes);
	        //   }
	        //   catch (FileNotFoundException ex){
	        //   	}
            
            URL wardsFile = MK_5.class.getResource
                    ("/Final_LSOA.shp");
            ShapeFileImporter.read(wardsFile, world, Polygon.class);
            System.out.println("	LSOA shapefile: " +wardsFile);
           
            MBR.expandToInclude(world.getMBR());

            // read in the FZ3 file
            URL flood3File = MK_5.class.getResource
            		("/NorfolkFZ3.shp");
            ShapeFileImporter.read(flood3File, flood3);
            System.out.println("	FZ3 shapefile: " +flood3File);

            MBR.expandToInclude(flood3.getMBR());

            // read in the FZ2 file
            URL flood2File = MK_5.class.getResource
            		("/NorfolkFZ2.shp");
            ShapeFileImporter.read(flood2File, flood2);
            System.out.println("	FZ2 shapefile: " +flood2File);

            MBR.expandToInclude(flood2.getMBR());
            
            /*
            // read in the household files
            URL HouseholdsFZFile = MK_2.class.getResource
            		("/data/Buildings_IN_FZ_Snapped_to_ITN.shp");
            ShapeFileImporter.read(HouseholdsFZFile, HouseholdsFZ);
            System.out.println("Households in FZ shapefile: " +HouseholdsFZFile);

            MBR.expandToInclude(HouseholdsFZ.getMBR());
            
            // read in the FZ2 file
            URL HouseholdsFile = MK_2.class.getResource
            		("/data/Buildings_NOT_in_FZ_Snapped_to_ITN.shp");
            ShapeFileImporter.read(HouseholdsFile, Households);
            System.out.println("Households not in FZ shapefile: " +HouseholdsFile);
            System.out.println();

            MBR.expandToInclude(Households.getMBR());
            */
            
            
            createNetwork();
            setup();
            

            //////////////////////////////////////////////
            ////////////////// CLEANUP ///////////////////
            //////////////////////////////////////////////

            // clear any existing agents from previous runs
            agents.clear();
            ngoagents.clear();
            elderlyagents.clear();
            limitedactionsagents.clear();

            //////////////////////////////////////////////
            ////////////////// AGENTS ////////////////////
            //////////////////////////////////////////////
            
            // initialize agents using the following source .CSV files
            populateAgent("/NorfolkITNAGENT.csv");
            populateNGO("/NorfolkITNNGO.csv");
            populateElderly("/NorfolkITNELDERLY.csv");
            populateLimitedActions("/NorfolkITNLIMITED.csv");
            System.out.println();
            System.out.println("Starting simulation...");

            // standardize the MBRs so that the visualization lines up
            // and everyone knows what the standard MBR is
            roads.setMBR(MBR);
            world.setMBR(MBR);
            flood3.setMBR(MBR);
            flood2.setMBR(MBR);
            //HouseholdsFZ.setMBR(MBR);
            //Households.setMBR(MBR);
            agents.setMBR(MBR);
            ngoagents.setMBR(MBR);
            elderlyagents.setMBR(MBR);
            limitedactionsagents.setMBR(MBR);

            // Ensure that the spatial index is updated after all the agents move
            schedule.scheduleRepeating( agents.scheduleSpatialIndexUpdater(),
            		Integer.MAX_VALUE, 1.0);
            schedule.scheduleRepeating( ngoagents.scheduleSpatialIndexUpdater(),
            		Integer.MAX_VALUE, 1.0);
            schedule.scheduleRepeating( elderlyagents.scheduleSpatialIndexUpdater(),
            		Integer.MAX_VALUE, 1.0);
            schedule.scheduleRepeating( limitedactionsagents.scheduleSpatialIndexUpdater(),
            		Integer.MAX_VALUE, 1.0);
            
            /**
             * Steppable that flips Agent paths once everyone reaches their destinations
             */
            Steppable flipper = new Steppable()	{
				private static final long serialVersionUID = 1L;

				@Override
                public void step(SimState state)	{

					MK_5 gstate = (MK_5) state;

					// checks to see if anyone has not yet reached destination
					// turning off makes agents move to next edge then back again
                    for (agents.MainAgent a : gstate.agentList)	{
                        if (!a.reachedDestination)	{	// someone is still moving: let them do so
                            return; 
                        }
                    }
                    // Now send everyone back in the opposite direction
                    boolean toWork = gstate.goToWork;
                    gstate.goToWork = !toWork;
                    // otherwise everyone has reached their latest destination:
                    // turn them back
                    // turning off means agents reach first destination and stay there.
                    for (agents.MainAgent a : gstate.agentList) 	{
                        a.flipPath();
                    }
                    
                    
                    // NGOAgent
                    for (NGOAgent b : gstate.ngoAgentList)	{
                        if (!b.reachedDestination)	{
                            return;
                        }
                    }
                    boolean toWork1 = gstate.goToWork1;
                    gstate.goToWork1 = !toWork1;
                    
                    for (NGOAgent b : gstate.ngoAgentList)	{
                        b.flipPath();
                    }
                    
                    
                    // ElderlyAgent
                    for (agents.ElderlyAgent c : gstate.elderlyAgentList)	{
                        if (!c.reachedDestination)	{
                            return;
                        }
                    }
                    boolean toWork2 = gstate.goToWork2;
                    gstate.goToWork2 = !toWork2;
                    
                    for (agents.ElderlyAgent c : gstate.elderlyAgentList)	{
                        c.flipPath();
                    }
                    
                    
                    // LimitedActionsAgent
                    for (LimitedActionsAgent d : gstate.limitedActionsAgentList)	{
                        if (!d.reachedDestination)	{
                            return;
                        }
                    }
                    boolean toWork3 = gstate.goToWork3;
                    gstate.goToWork3 = !toWork3;
                    
                    for (LimitedActionsAgent d : gstate.limitedActionsAgentList)	{
                        d.flipPath();
                    }
                }
            };
            schedule.scheduleRepeating(flipper, 10);
            // 10? Does it repeat 10 times? Appears to go on forever...

        } catch (FileNotFoundException e)	{
            System.out.println("Error: missing required data file");
        }
    }


    /**
	 * Finish the simulation and clean up
	 */
    public void finish()	{
    	super.finish();
    	System.out.println();
    	System.out.println("Simulation ended by user.");
        /*
    	System.out.println("Attempting to export agent data...");
        try	{
        	ShapeFileExporter.write("agents", agents);
        } catch (Exception e)	{
        	System.out.println("Export failed.");
        	e.printStackTrace();
        }
        */
    }


    /**
     * Create the road network the agents will traverse
     */
    private void createNetwork()	{
    	System.out.println("Creating road network..." +roads);
    	System.out.println();
    	network.createFromGeomField(roads);

        for (Object o : network.getEdges())	{
            GeomPlanarGraphEdge e = (GeomPlanarGraphEdge) o;
            
            //System.out.println("idsToEdges = " +idsToEdges);
            idsToEdges.put(e.getIntegerAttribute("ROAD_ID").intValue(), e);
            //System.out.println("idsToEdges = " +idsToEdges);

            e.setData(new ArrayList<agents.MainAgent>());
        }

        addIntersectionNodes(network.nodeIterator(), junctions);
    }
    
    
    /**
     * Read in the population files and create appropriate populations
     * @param filename
     */
    ////////////////////// MainAgent /////////////////////////////
    public void populateAgent(String filename)	{
    	//System.out.println("Populating model: ");
        try	{
            String filePath = MK_5.class.getResource(filename).getPath();
            FileInputStream fstream = new FileInputStream(filePath);
            System.out.println("Populating model with Main Agents: " +filePath);

            BufferedReader d = new BufferedReader(new InputStreamReader(fstream));
            String s;

            // get rid of the header
            d.readLine();
            // read in all data
            while ((s = d.readLine()) != null)	{
                String[] bits = s.split(",");

                int pop = Integer.parseInt(bits[2]);
                //System.out.println();
                System.out.println("Main Agent road segment population (C:Count): " +pop);

                String homeTract = bits[3];
                System.out.println("Main Agent homeTract (D:ROAD_ID): " +homeTract);

                String workTract = bits[4];
                System.out.println("Main Agent workTract (E:Work): " +workTract);

                String id_id = bits[3];
                System.out.println("Main Agent ID_ID (D:ROAD_ID): " +id_id);

                String ROAD_ID = bits[3];
                System.out.println("Main Agent road segment (D:ROAD_ID): " +ROAD_ID);
                //System.out.println();
                
                GeomPlanarGraphEdge startingEdge = idsToEdges.get(
                	(int) Double.parseDouble(ROAD_ID));
                GeomPlanarGraphEdge goalEdge = idsToEdges.get(
            		//(int) Double.parseDouble(workTract));	//reads the .CSV column
                    goals[ random.nextInt(goals.length)]); // uses the hardcoded 'goals' from above
                //System.out.println("startingEdge: " +startingEdge);
                //System.out.println("idsToEdges: " +idsToEdges);
                //System.out.println("goalEdge: " +goalEdge);
                //System.out.println("goals: " +goals);
                //System.out.println("homeNode: " +goals);
                //System.out.println();

                for (int i = 0; i < pop; i++)	{
                	//pop; i++)	{ 	// NO IDEA IF THIS MAKES A DIFFERENCE!?!
                    agents.MainAgent a = new agents.MainAgent(this, homeTract, workTract, startingEdge, goalEdge);
                    /*System.out.println(
                    		"Main Agent " + i + ":	" 
            				+ " Home: " +homeTract + ";	"
                    		+ "	Work: " +workTract + ";	"
                    		//+ " Starting Edge: " +startingEdge + ";"
                    		+ "	Pop: " +pop + ";	"
                    		+ "	id_id: " +id_id + ";	"
                    		+ "	Road_ID: " +ROAD_ID);*/
                    boolean successfulStart = a.start(this);
                    //System.out.println("Starting...");

                    if (!successfulStart)	{
                    	System.out.println("Main agents added successfully!!");
                    	continue; // DON'T ADD IT if it's bad
                    }

                    // MasonGeometry newGeometry = new MasonGeometry(a.getGeometry());
                    MasonGeometry newGeometry = a.getGeometry();
                    newGeometry.isMovable = true;
                    agents.addGeometry(newGeometry);
                    agentList.add(a);
                    schedule.scheduleRepeating(a);
                }
            }

            d.close();

        } catch (Exception e) {
		    	System.out.println("ERROR: issue with population file: ");
				e.printStackTrace();
		}
    }

    //////////////////////// NGOAgent /////////////////////////////

    public void populateNGO(String filename)	{
        try	{
            String filePath = MK_5.class.getResource(filename).getPath();
            FileInputStream fstream = new FileInputStream(filePath);
            System.out.println();
            System.out.println("Populating model with NGO Agents: " +filePath);

            BufferedReader d = new BufferedReader(new InputStreamReader(fstream));
            String s;

            // get rid of the header
            d.readLine();
            // read in all data
            while ((s = d.readLine()) != null)	{
                String[] bits = s.split(",");
                
                int pop = Integer.parseInt(bits[2]);
                //System.out.println();
                System.out.println("NGO Agent road segment population (C:Count): " +pop);

                String homeTract = bits[3];
                System.out.println("NGO Agent homeTract (D:ROAD_ID): " +homeTract);

                String workTract = bits[4];
                System.out.println("NGO Agent workTract (E:Work): " +workTract);

                String id_id = bits[3];
                System.out.println("NGO Agent ID_ID (D:ROAD_ID): " +id_id);

                String ROAD_ID = bits[3];
                System.out.println("NGO Agent road segment (D:ROAD_ID): " +ROAD_ID);
                //System.out.println();

                GeomPlanarGraphEdge startingEdge = idsToEdges.get(
                		(int) Double.parseDouble(ROAD_ID));
                GeomPlanarGraphEdge goalEdge = idsToEdges.get(
                		(int) Double.parseDouble(workTract));
                    //goals1[ random.nextInt(goals1.length)]);

                for (int i = 0; i < pop; i++)	{
                	//pop; i++)	{ 	// NO IDEA IF THIS MAKES A DIFFERENCE!?!
                    agents.NGOAgent a = new agents.NGOAgent(this, homeTract, workTract, startingEdge, goalEdge);
                    /*System.out.println(
                    		"NGO Agent " + i + ":	" 
            				+ " Home: " +homeTract + ";	"
                    		+ "	Work: " +workTract + ";	"
                    		//+ " Starting Edge: " +startingEdge + ";"
                    		+ "	Pop: " +pop + ";	"
                    		+ "	id_id: " +id_id + ";	"
                    		+ "	Road_ID: " +ROAD_ID);*/
                    boolean successfulStart = a.start(this);

                    if (!successfulStart)	{
                    	System.out.println("NGO agents added successfully!!");
                    	continue; // DON'T ADD IT if it's bad
                    }

                    MasonGeometry newGeometry = a.getGeometry();
                    newGeometry.isMovable = true;
                    ngoagents.addGeometry(newGeometry);
                    ngoAgentList.add(a);
                    schedule.scheduleRepeating(a);
                }
            }

            d.close();

        } catch (Exception e) {
		    	System.out.println("ERROR: issue with population file: ");
				e.printStackTrace();
		}
    }

    //////////////////////// ElderlyAgent ///////////////////////////
    public void populateElderly(String filename)	{
        try	{
            String filePath = MK_5.class.getResource(filename).getPath();
            FileInputStream fstream = new FileInputStream(filePath);
            System.out.println();
            System.out.println("Populating model with Elderly Agents: " +filePath);

            BufferedReader d = new BufferedReader(new InputStreamReader(fstream));
            String s;

            // get rid of the header
            d.readLine();
            // read in all data
            while ((s = d.readLine()) != null)	{
                String[] bits = s.split(",");

                int pop = Integer.parseInt(bits[2]);
                //System.out.println();
                System.out.println("Elderly Agent road segment population (C:Count): " +pop);

                String homeTract = bits[3];
                System.out.println("Elderly Agent homeTract (D:ROAD_ID): " +homeTract);

                String workTract = bits[4];
                System.out.println("Elderly Agent workTract (E:Work): " +workTract);

                String id_id = bits[3];
                System.out.println("Elderly Agent ID_ID (D:ROAD_ID): " +id_id);

                String ROAD_ID = bits[3];
                System.out.println("Elderly Agent road segment (D:ROAD_ID): " +ROAD_ID);
                //System.out.println();

                GeomPlanarGraphEdge startingEdge = idsToEdges.get(
                		(int) Double.parseDouble(ROAD_ID));
                GeomPlanarGraphEdge goalEdge = idsToEdges.get(
                		(int) Double.parseDouble(workTract));
                //goals2[ random.nextInt(goals2.length)]);

                for (int i = 0; i < pop; i++)	{
                	//pop; i++)	{ 	// NO IDEA IF THIS MAKES A DIFFERENCE!?!
                    agents.ElderlyAgent a = new agents.ElderlyAgent(this, homeTract, workTract, startingEdge, goalEdge);
                    /*System.out.println(
                    		"Elderly Agent " + i + ":	" 
            				+ " Home: " +homeTract + ";	"
                    		+ "	Work: " +workTract + ";	"
                    		//+ " Starting Edge: " +startingEdge + ";"
                    		+ "	Pop: " +pop + ";	"
                    		+ "	id_id: " +id_id + ";	"
                    		+ "	Road_ID: " +ROAD_ID);*/
                    boolean successfulStart = a.start(this);

                    if (!successfulStart)	{
                    	System.out.println("Elderly agents added successfully!");
                    	continue; // DON'T ADD IT if it's bad
                    }

                    MasonGeometry newGeometry = a.getGeometry();
                    newGeometry.isMovable = true;
                    elderlyagents.addGeometry(newGeometry);
                    elderlyAgentList.add(a);
                    schedule.scheduleRepeating(a);
                }
            }

            d.close();

        } catch (Exception e) {
		    	System.out.println("ERROR: issue with population file: ");
				e.printStackTrace();
		}
    }

    //////////////////// LimitedActions Agent ////////////////////////////
    public void populateLimitedActions(String filename)	{
        try	{
            String filePath = MK_5.class.getResource(filename).getPath();
            FileInputStream fstream = new FileInputStream(filePath);
            System.out.println();
            System.out.println("Populating model with Limited Actions Agents: " +filePath);

            BufferedReader d = new BufferedReader(new InputStreamReader(fstream));
            String s;

            // get rid of the header
            d.readLine();
            // read in all data
            while ((s = d.readLine()) != null)	{
                String[] bits = s.split(",");

                int pop = Integer.parseInt(bits[2]);
                //System.out.println();
                System.out.println("LimitedActions Agent road segment population (C:Count): " +pop);

                String homeTract = bits[3];
                System.out.println("LimitedActions Agent homeTract (D:ROAD_ID): " +homeTract);

                String workTract = bits[4];
                System.out.println("LimitedActions Agent workTract (E:Work): " +workTract);

                String id_id = bits[3];
                System.out.println("LimitedActions Agent ID_ID (D:ROAD_ID): " +id_id);

                String ROAD_ID = bits[3];
                System.out.println("LimitedActions Agent road segment (D:ROAD_ID): " +ROAD_ID);
                //System.out.println();

                GeomPlanarGraphEdge startingEdge = idsToEdges.get(
                		(int) Double.parseDouble(ROAD_ID));
                GeomPlanarGraphEdge goalEdge = idsToEdges.get(
                		(int) Double.parseDouble(workTract));
                //goals3[ random.nextInt(goals3.length)]);

                for (int i = 0; i < pop; i++)	{
                	//pop; i++)	{ 	// NO IDEA IF THIS MAKES A DIFFERENCE!?!
                    LimitedActionsAgent a = new LimitedActionsAgent(this, homeTract, workTract, startingEdge, goalEdge);
                    /*System.out.println(
                    		"Limited Actions Agent " + i + ":	" 
            				+ " Home: " +homeTract + ";	"
                    		+ "	Work: " +workTract + ";	"
                    		//+ " Starting Edge: " +startingEdge + ";"
                    		+ "	Pop: " +pop + ";	"
                    		+ "	id_id: " +id_id + ";	"
                    		+ "	Road_ID: " +ROAD_ID);*/
                    boolean successfulStart = a.start(this);

                    if (!successfulStart)	{
                    	System.out.println("Limited Actions agents successfully added!");
                    	continue; // DON'T ADD IT if it's bad
                    }

                    MasonGeometry newGeometry = a.getGeometry();
                    newGeometry.isMovable = true;
                    limitedactionsagents.addGeometry(newGeometry);
                    limitedActionsAgentList.add(a);
                    schedule.scheduleRepeating(a);
                }
            }

            d.close();

        } catch (Exception e) {
		    	System.out.println("ERROR: issue with population file: ");
				e.printStackTrace();
		}
    }

    /** adds nodes corresponding to road intersections to GeomVectorField
     *
     * @param nodeIterator Points to first node
     * @param intersections GeomVectorField containing intersection geometry
     *
     * Nodes will belong to a planar graph populated from LineString network.
     */
    private void addIntersectionNodes(Iterator<?> nodeIterator,
                                      GeomVectorField intersections)	{
        GeometryFactory fact = new GeometryFactory();
        Coordinate coord = null;
        Point point = null;
        @SuppressWarnings("unused")
		int counter = 0;

        while (nodeIterator.hasNext())	{
            Node node = (Node) nodeIterator.next();
            coord = node.getCoordinate();
            point = fact.createPoint(coord);

            junctions.addGeometry(new MasonGeometry(point));
            counter++;
        }
    }


    /**
     * Main function allows simulation to be run in stand-alone, non-GUI mode
     */
    public static void main(String[] args)	{
        doLoop(MK_5.class, args);
        System.exit(0);
    }
}
