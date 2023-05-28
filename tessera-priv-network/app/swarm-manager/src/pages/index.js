import Head from 'next/head'
import { Inter } from 'next/font/google'
import styles from '@/styles/Home.module.css'
import { useMoralis } from "react-moralis";
import Swal from "sweetalert2";
import { ConnectButton } from 'web3uikit';
import Container from 'react-bootstrap/Container';
import Nav from 'react-bootstrap/Nav';
import Navbar from 'react-bootstrap/Navbar';
import { useEffect, useState } from 'react';
import { routeApiHandler } from '../../utils/apiUtils';
import Modal from 'react-bootstrap/Modal';
import Button from 'react-bootstrap/Button';
import ManageAction from '../../components/manageAction';
import {AiOutlineReload} from 'react-icons/ai';
import Spinner from 'react-bootstrap/Spinner';


const truncateStr = (fullStr, strLen) => {
  if (fullStr.length <= strLen) return fullStr;

  const separator = "...";
  let separatorLength = separator.length;
  const charToShow = strLen - separatorLength;
  const frontChars = Math.ceil(charToShow / 2);
  const backChars = Math.floor(charToShow / 2);
  return (
    fullStr.substring(0, frontChars) +
    separator +
    fullStr.substring(fullStr.length - backChars)
  );
};
const inter = Inter({ subsets: ['latin'] })

export default function Home() {

  const { isWeb3Enabled, account } = useMoralis();
  const [managmentType, setManagmentType] = useState(-1);
  const [routes, setRoutes] = useState([]);
  const [show, setShow] = useState(false);
  const [title, setTitle] = useState();
  const [description, setDescription] = useState();
  const [activeRoute, setActiveRoute] = useState(0);
  const [refresh, setRefresh] = useState(false);
  const [routeInfo, setRouteInfo] = useState([]);
  const [loading, setLoading] = useState(false);
  
  const handleClose = () => setShow(false);

  const handleShow = (type) => {
    setShow(true)
    if(type === "add route"){
      setTitle("Add route")
      setDescription("Add a route to the swarm")
    }else if(type === "remove route"){
      setTitle("Remove route")
      setDescription("Remove a route from the swarm")
    }else if(type === "add node"){
      setTitle("Add node")
      setDescription("Add a node to the swarm")
    }else if(type === "remove node"){
      setTitle("Remove node")
      setDescription("Remove a node from the swarm")
    }else if(type === "add account"){
      setTitle("Add account")
      setDescription("Add a new account to the swarm")
    }else if(type === "remove account"){
      setTitle("Remove account")
      setDescription("Remove an account from the swarm")
    }
  };

  //make a swal function to display when error on api requests
  const errorSwal = () => {
    Swal.fire({
      title: 'ERROR',
      text: "Can't interact with the blockchain - security error",
      icon: "error",
      position: "center",
      color: "black",
      showConfirmButton: false,
    });
    setRoutes([]);
  }


  const checkRoutesAvailable = async (account) => {
    //make a swal that says loading routes
    const swal = Swal.fire({
      title: 'Checking routes...',
      html: 'Please wait',
      allowOutsideClick: false,
      showConfirmButton: false,
      didOpen: () => {
        Swal.showLoading()
      }
    })

    await routeApiHandler('getRoutes',account).then((result) => {
      swal.close();
      if(result === undefined){
        errorSwal()
      }else{
        setRoutes(result);
      }
    }).catch((error) => {
      console.log(error)
    })
  }

  const checkRouteInfo = async (route, routeId) => {
    setActiveRoute(routeId);
    let params = route.routeName.split(" - ")
    const swal = Swal.fire({
      title: 'Checking route info...',
      html: 'Please wait',
      allowOutsideClick: false,
      showConfirmButton: false,
      didOpen: () => {
        Swal.showLoading()
      }
    })
    await routeApiHandler('getRouteInfo',account, params[1], params[0]).then((result) => {
      swal.close();
      if(result === undefined){
        errorSwal()
      }else{
        setRouteInfo(result);
        console.log(result)
      }
    }).catch((error) => {
      console.log(error)
    }
    )
    
  }
  

  useEffect(() => {
    if(refresh){

      setTimeout(() => {
        setRefresh(false);
        checkRoutesAvailable(account);
        console.log("refreshing")
      }, 1500);
    }
  }, [refresh])

  async function refreshRouteData(){
    setLoading(true);
    let params = routes[activeRoute].routeName.split(" - ")
    await routeApiHandler('getRouteInfo',account, params[1], params[0])
                      .then((result) => {
                        setRouteInfo(result);
                        setLoading(false);
                      }).catch
                      ((error) => {
                        console.log(error)
                      }
                      )
  }
  
  const dropSwal = () =>{
    Swal.fire({
      title: `¡You are in!`,
      timer: 1700,
      icon: "success",
      position: "center",
      color: "white",
      showConfirmButton: false,
    });
  }

  return (
    <>
      <Head>
        <title>Swarm Manager</title>
        <link rel="icon" href="/icon.jpg" />
        <meta name="description" content="Generated by create next app" />
        <meta name="viewport" content="width=device-width, initial-scale=1" />
        <link rel="preconnect" href="https://fonts.googleapis.com"/>
        <link rel="preconnect" href="https://fonts.gstatic.com" crossOrigin="true"/>
        <link href="https://fonts.googleapis.com/css2?family=Roboto+Condensed&display=swap" rel="stylesheet"/>
      </Head>
      <main className={styles.main}>
        <h1 className={styles.title}>
          Swarm Manager Powered by Blockchain Technology
        </h1>
        
        <p>Author - Carlos Rocamora Esteban</p>
        {isWeb3Enabled ? (
              <img className='center' width={"100px"} src='/download.png'></img>
        ) : (
         
            '')
            }
        
        {isWeb3Enabled ? (
          <div>
              <Navbar bg="none" expand="lg">
                <Container>
                  <Navbar.Toggle aria-controls="basic-navbar-nav" />
                  <Navbar.Collapse id="basic-navbar-nav">
                    <Nav className="me-auto">
                      <div className='m-2'>
                        <ConnectButton moralisAuth={true} chainId={1337}></ConnectButton>
                      </div>
                      <a type='button' className={'p-2 m-2 btn btn-info'} onClick={()=>{setManagmentType(0)}}>Route managment</a>
                      <a type='button' className='p-2 m-2 btn btn-warning' onClick={()=>{setManagmentType(1)}}>Node swarm permissions</a>
                      <a type='button' className='p-2 m-2 btn btn-warning' onClick={()=>{setManagmentType(2)}}>Accounts swarm permissions</a>
                    </Nav>
                  </Navbar.Collapse>
                </Container>
              </Navbar>
              <>
              <hr></hr>
                {managmentType === 0 ? (
                  <div>
                    <h3>
                      Route managment
                    </h3>
                    <div className='m-2'>
                      <a type='button' className='p-2 m-2 btn btn-warning' onClick={()=>{checkRoutesAvailable(account)}}>Check routes</a>
                      <a type='button' className='p-2 m-2 btn btn-success' onClick={()=>{handleShow('add route')}}>Add route</a>
                      <a type='button' className='p-2 m-2 btn btn-danger' onClick={()=>{handleShow('remove route')}}>Remove route</a>
                    </div>
                    <hr className='h-1 bg-light'></hr>
                    <div className='m-2 bg-dark p-2'>
                      <h4 className='m-2'>Routes</h4>
                      {routes && routes.length>0 ? routes.map((route, index) => {
                        return(
                          <a id={'route'+index} key={index} type='button' className={activeRoute===index ? 'btn m-2 btn-primary':'btn m-2 btn-secondary'} 
                              onClick={()=>{checkRouteInfo(route, index)}}>{index+1}: {route.routeName}</a>
                          )
                      }, this):''}
                      <hr></hr>
                      {routes && routes.length>0 ?
                        <div>
                          <h4 className='m-2'>Route info</h4>
                          <div className='m-2'>
                            <p>Route name: {routes[activeRoute].routeName}</p>
                            <p>Route address: {routes[activeRoute].address}</p>
                            <hr></hr>
                            <div className='mb-3'><b className='mx-2'>Refresh</b> 
                              {loading ? <Spinner animation="border" role="status">
                                          <span className="visually-hidden">Loading...</span>
                                        </Spinner> : <a className='btn btn-secondary p-1' onClick={refreshRouteData}><AiOutlineReload /></a>}
                              </div>
                            <p>Route status: {routeInfo.status}</p>
                            <p>Cars in route:</p>
                           {routeInfo.cars && routeInfo.cars.length>0 ?
                            
                              <ul>
                                {routeInfo.cars.map((car, index) => {
                                  return(
                                    <li key={index}>{car}</li>
                                  )
                                })}
                              </ul>
                            
                           : <p>No active cars in the route</p>}
                          </div>
                        </div>
                        
                        :''
                      }
                      
                    </div>
                  </div>
                  
                  
                ) : managmentType === 1 ? (
                  <div>
                    <h3>
                      Node swarm permissions
                    </h3>
                    <div className='m-2'>
                      <a type='button' className='p-2 m-2 btn btn-success' onClick={()=>{handleShow('add node')}} >Add node</a>
                      <a type='button' className='p-2 m-2 btn btn-danger' onClick={()=>{handleShow('remove route')}} >Remove node</a>
                    </div>
                  </div>
                ) : managmentType === 2 ? (
                  <div>
                    <h3>
                      Accounts swarm permissions
                    </h3>
                    <div className='m-2'>
                      <a type='button' className='p-2 m-2 btn btn-success' onClick={()=>{handleShow('add account')}} >Add account</a>
                      <a type='button' className='p-2 m-2 btn btn-danger' onClick={()=>{handleShow('remove account')}} >Remove account</a>
                    </div>
                  </div>
                ) : (
                  <div className='justify-items-center'>
                    <h3>
                      Welcome to the Swarm Manager, please select an option to start managing your swarm.
                    </h3>
                  </div>
                )}
              </>
              <Modal size="lg" centered show={show} onHide={handleClose}>
                <Modal.Header closeButton>
                  <Modal.Title>{title}</Modal.Title>
                </Modal.Header>
                <Modal.Body>
                  {description} 
                  <ManageAction type={title} account={account} show={setShow} routes={routes} update={setRefresh}></ManageAction>
                </Modal.Body>
                <Modal.Footer>
                  <Button variant="secondary" onClick={handleClose}>
                    Close
                  </Button>
                </Modal.Footer>
              </Modal>
          </div>
        ) : (
          <div>
              <img width={'700px'} src="/icon.jpg"></img>
              <p className='text-center'><b>Image generated by Deep Dream Generator</b></p>
                <div className={styles.button}>
                  <ConnectButton moralisAuth={false} id="connecButton" chainId={1337}></ConnectButton>
                </div>
          </div>
        )}
      </main>
    </>
  )
}
