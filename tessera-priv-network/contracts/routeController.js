const path = require('path');
const fs = require('fs-extra');
const Web3 = require('web3');
const Web3Quorum = require('web3js-quorum');
const chainId = 1337;
const contractByteCode = fs.readFileSync(__dirname+'/RouteFactory_sol_RouteFactory.bin');
const contractAbi = JSON.parse(fs.readFileSync(__dirname+'/RouteFactory_sol_RouteFactory.abi'));

const { tessera, besu } = require("../app/keys.js");


// WARNING: the keys here are demo purposes ONLY. Please use a tool like Orchestrate or EthSigner for production, rather than hard coding private keys
async function deployContract(clientUrl, fromPrivateKey, fromPubKey, toPublicKey){
    const contractConstructorInit = "000000000000000000000000000000000000000000000000000000000000002F";
    const web3 = new Web3(clientUrl)
    const web3quorum = new Web3Quorum(web3, chainId);
    // const contractByteCode = fs.readFileSync(__dirname+'/RouteFactory_sol_RouteFactory.bin');

    // const contractAbi = JSON.parse(fs.readFileSync(__dirname+'/'+contractName+'_sol_'+contractName+'.abi'));

    const txOptions = {
        data: '0x'+contractByteCode+contractConstructorInit,
        privateKey: fromPrivateKey,
        privateFrom: fromPubKey,
        privateFor: [toPublicKey]
    };
    
    const txHash = await web3quorum.priv.generateAndSendRawTransaction(txOptions);
    const privateTxReceipt = await web3quorum.priv.waitForTransactionReceipt(txHash);
    
    return privateTxReceipt;
}

async function createRoute(clientUrl, address, location, name, fromPrivateKey, fromPublicKey, toPublicKey){
    const web3 = new Web3(clientUrl)
    const web3quorum = new Web3Quorum(web3, chainId);
    const contract = new web3quorum.eth.Contract(contractAbi);
    // eslint-disable-next-line no-underscore-dangle
    const functionAbi = contract._jsonInterface.find(e => {
        return e.name === "createRoute";
    });

    const functionArgs = web3quorum.eth.abi
    .encodeParameters(functionAbi.inputs, [location, name])
    .slice(2);

    const functionParams = {
        to: address,
        data: functionAbi.signature + functionArgs,
        privateKey: fromPrivateKey,
        privateFrom: fromPublicKey,
        privateFor: [toPublicKey]
    };
    
    const transactionHash = await web3quorum.priv.generateAndSendRawTransaction(functionParams);
    console.log(`Route contract deployed by factory for ${location}, ${name}`)
    const result = await web3quorum.priv.waitForTransactionReceipt(transactionHash);

    return result;
}

async function setUp(){
    console.log('############################################################')
    console.log('# Setting up Blockchain Swarm Traffic Manager, please wait...')
    console.log('############################################################')
    let contract = await deployContract(besu.member1.url, besu.member1.accountPrivateKey, tessera.member1.publicKey, tessera.member3.publicKey)
    console.log('############################################################')
    console.log(`# Main route factory smart contract deployed at ${contract.contractAddress}, adding main route "Alicante, AP-7" to the manager`)
    console.log('############################################################')
    await createRoute(besu.member1.url, contract.contractAddress, 'Alicante','AP-7', besu.member1.accountPrivateKey, tessera.member1.publicKey, tessera.member3.publicKey)
    console.log('############################################################')
    
    return contract.contractAddress;
}

async function routeHandler(method, location, name, clientUrl, address, fromPrivateKey, fromPublicKey, toPublicKey, user, routeId){
    const web3 = new Web3(clientUrl)
    const web3quorum = new Web3Quorum(web3, chainId);
    const contract = new web3quorum.eth.Contract(contractAbi);
    const functionAbi = contract._jsonInterface.find(e => {
        return e.name === method;
    });
    
    const functionArgs = web3quorum.eth.abi
    .encodeParameters(functionAbi.inputs, [routeId])
    .slice(2);

    const functionParams = {
        to: address,
        data: functionAbi.signature + functionArgs,
        privateKey: fromPrivateKey,
        privateFrom: fromPublicKey,
        privateFor: [toPublicKey]
    };
       
    const transactionHash = await web3quorum.priv.generateAndSendRawTransaction(functionParams);
    console.log(`${method} completed -> ${location}, ${name} : ${user}`)
    const result = await web3quorum.priv.waitForTransactionReceipt(transactionHash);

    return result;
}

async function getRouteIndex(location, name, clientUrl, address, fromPrivateKey, fromPublicKey, toPublicKey){
    const web3 = new Web3(clientUrl)
    const web3quorum = new Web3Quorum(web3, chainId);
    const contract = new web3quorum.eth.Contract(contractAbi);
    // eslint-disable-next-line no-underscore-dangle
    const functionAbi = contract._jsonInterface.find(e => {
        return e.name === "getRouteIndex";
    });

    const functionArgs = web3quorum.eth.abi
    .encodeParameters(functionAbi.inputs, [location,name])
    .slice(2);

    const functionParams = {
        to: address,
        data: functionAbi.signature + functionArgs,
        privateKey: fromPrivateKey,
        privateFrom: fromPublicKey,
        privateFor: [toPublicKey]
    };

    const transactionHash = await web3quorum.priv.generateAndSendRawTransaction(functionParams);
    console.log("Route index is: " + result.output);

    const result = await web3quorum.priv.waitForTransactionReceipt(transactionHash);
    return result.output;
}




module.exports = {
    deployContract, 
    createRoute, 
    setUp, 
    routeHandler, 
    getRouteIndex, 
    
}
