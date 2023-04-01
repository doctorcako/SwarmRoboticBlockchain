const { besu } = require("../../appKeys.js");

async function routeApiHandler(method,account, location = null, routeName = null) {
    
    let nodeSender = besu[`${account}`]

    let nodeReceivers = []
    Object.values(besu).forEach((node) => {
       if(node.accountAddress != nodeSender.accountAddress){
           nodeReceivers.push(node)
       }
    })

    let publicKeys = []
    nodeReceivers.forEach((node) => {
        if(node.publicKey)
            publicKeys.push(node.publicKey)
    })
    
    const data = {
        nodeRPC: nodeSender.url,
        fromPrivateKey: nodeSender.accountPrivateKey,
        fromPublicKey: nodeSender.publicKey,
        toPublicKey: publicKeys
    };

    if(location && routeName){
        data.location = location
        data.routeName = routeName
    }

    const response = await fetch(`http://localhost:4000/${method}`, {
        headers: {
            'content-type': 'application/json'
        },
        method: "POST",
        body: JSON.stringify(data)
    }).catch((err) => {
        console.log('Api failure: '+err)
    });


    let result;
    if(response.status == 200){
        result = await response.json();
    }
    

    return result;

}



module.exports = {
    routeApiHandler,
}