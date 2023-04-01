import InputGroup from 'react-bootstrap/InputGroup';
import Form from 'react-bootstrap/Form';
import { routeApiHandler } from '../utils/apiUtils';
import Swal from "sweetalert2";
import { useState } from 'react';

export default function ManageAction({type, account, show, routes, update}){

    const routesReceived = routes;

    function AddRoute(account){

        const saveRoute = async (account) => {

            let location = document.getElementById("routeLocation").value
            let routeName = document.getElementById("routeName").value

            if(location == "" || routeName == ""){
                Swal.fire({
                    title: `¡Please fill all the fields!`,
                    icon: "error",
                    position: "center",
                    color: "black",
                    showConfirmButton: true,
                    })
                return

            }else{
                Swal.fire({
                    title: `¡Route added!`,
                    timer: 1700,
                    icon: "success",
                    position: "center",
                    color: "black",
                    showConfirmButton: false,
                    }).then(async () => {
                        show(false)
                        update(true)
                        await routeApiHandler('createNewRoute', account.account, location, routeName)
                    });
            }
        }

        return (
            <>
                <div className="flex text-center col-12">
                    <InputGroup className="mb-3">
                    <Form.Control
                        required
                        id="routeLocation"
                        placeholder="Location of the route. Example: Alicante"
                        aria-label="Name"
                        aria-describedby="basic-addon1"
                        />
                    </InputGroup>

                    <InputGroup className="mb-3">
                    <Form.Control
                        required
                        id="routeName"
                        placeholder="Name of the route. Example: AP-7"
                        aria-label="Username"
                        aria-describedby="basic-addon1"
                        />
                    </InputGroup>
                    <p className="mt-3 text-danger">The examples already exists by default!</p>
                    <button className="btn btn-success" onClick={()=>{saveRoute(account)}}>Save</button>
                </div>
            </>
        )
    }

    function RemoveRoute(account){
        console.log(routesReceived)
        return (
            <>
            {
                routesReceived && routesReceived.length>0 ?
                <div className="flex text-center col-12">
                    <select className="form-select " aria-label="Default select example">
                        <option selected>Open this select menu</option>
                        {
                            routes.map((route, index) => {
                                return (
                                    <option key={index} value={route.address}>{route.routeName}</option>
                                )
                            })
                        }
                    </select>
                    <button className="btn btn-danger">Remove</button>
                </div>
                :
                <h3>No routes to remove</h3>
            }
            </>
        )
    }

    function AddNode(account){
        return (
            <>
            <h1>Add Node</h1>
            </>
        )
    }

    function RemoveNode(account){
        return (
            <>
            <h1>Remove Node</h1>
            </>
        )
    }

    function AddAccount(account){
        return (
            <>
            <h1>Add Account</h1>
            </>
        )
    }

    function RemoveAccount(account){
        return (
            <>
            <h1>Remove Account</h1>
            </>
        )
    }


    return (
        <>
        {
            type === "Add route" ? <AddRoute account={account}/> : 
            type === "Remove route" ? <RemoveRoute account={account.account} routes={routes}/> :
            type === "Add node" ? <AddNode account={account.account}/> :
            type === "Remove node" ? <RemoveNode account={account.account}/> :
            type === "Add account" ? <AddAccount account={account.account}/> :
            type === "Remove account" ? <RemoveAccount account={account.account}/> :
        null} 
        </>
        )


}